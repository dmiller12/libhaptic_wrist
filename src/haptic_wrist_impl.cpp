#include "haptic_wrist_impl.h"
#include "haptic_wrist/haptic_wrist_config.h"
#include "utils.h"
#include "yaml-cpp/yaml.h"
#include <boost/filesystem.hpp>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include "haptic_wrist/trajectory.h"
#include "config_loader.h"

using namespace mjbots;

namespace haptic_wrist {

HapticWristImpl::HapticWristImpl()
    : handle_theta_(jp_type::Zero())
    , handle_dtheta_(jv_type::Zero())
    , handle_torque_(jt_type::Zero())
    , handle_orientation_(Eigen::Quaterniond::Identity())
    , orientation_des_(Eigen::Quaterniond::Identity())
    , control_period_(1.0 / control_rate_) {


    std::string config_dir = get_config_directory();
    if (config_dir.empty()) {
        throw std::runtime_error("No valid configuration directory found.");
    }

    HapticWristConfig config = load_config(config_dir);

    home_ = config.home_position;

    kinematics_ = Kinematics(config.dh_parameters, config.eef_to_tool, Eigen::Matrix4d::Identity());
    gravity_compensator_ = GravityComp(config.gravity_mus);

    orientation_controller_ =
        std::make_unique<OrientationController>(config.orientation_controller.kp, config.orientation_controller.kd);

    joint_position_controller_ = std::make_unique<JointPositionController>(
        config.joint_position_controller.kp, config.joint_position_controller.kd, control_period_.count());

    // Transformation matrices for motor/joint conversions
    jtmp_matrix_ = config.j2mp;
    mtjp_matrix_ = jtmp_matrix_.inverse();

    // Configure Moteus controllers for TORQUE control
    moteus::Controller::Options options_common;
    auto& pf = options_common.position_format;
    pf.position = moteus::kIgnore;
    pf.velocity = moteus::kIgnore; // Defaults to zero, no need to send
    pf.feedforward_torque = moteus::kFloat;
    // defaults to one, modify the kp and kd directly
    pf.kd_scale = moteus::kIgnore; 
    pf.kp_scale = moteus::kIgnore; 
    
    auto& qf = options_common.query_format;
    qf.voltage = moteus::kIgnore;
    qf.temperature = moteus::kIgnore;

    auto args = moteus::Controller::ProcessTransportArgs(config.moteus.transport_args);

    transport_ = moteus::Controller::MakeSingletonTransport({});

    controllers_ = {
        std::make_shared<moteus::Controller>([&]() { auto opts = options_common; opts.id = 1; return opts; }()),
        std::make_shared<moteus::Controller>([&]() { auto opts = options_common; opts.id = 2; return opts; }()),
        std::make_shared<moteus::Controller>([&]() { auto opts = options_common; opts.id = 3; return opts; }()),
        std::make_shared<moteus::Controller>([&]() { auto opts = options_common; opts.id = 4; return opts; }())
    };

    // Set moteus params and initialize motors to a stopped state
    size_t i = 0;
    for (auto& c : controllers_) {
        c->DiagnosticWrite("tel stop\n");
        c->DiagnosticFlush();
        std::ostringstream ostr;
        ostr << "conf set servo.pid_position.kp " << 0;
        c->DiagnosticCommand(ostr.str());
        ostr << "conf set servo.pid_position.ki " << 0;
        c->DiagnosticCommand(ostr.str());
        ostr << "conf set servo.pid_position.kd " << config.moteus.kd(i);
        c->DiagnosticCommand(ostr.str());
        c->SetStop();
        ++i;
    }
};

HapticWristImpl::~HapticWristImpl() {
    stop();
}

void HapticWristImpl::setTarget(const Eigen::Quaterniond& orientation) {
    boost::lock_guard<boost::mutex> lock(set_mutex_);
    orientation_des_ = orientation.normalized();
    control_mode_.store(ControlMode::ORIENTATION);
};

void HapticWristImpl::setTarget(const jp_type& position) {
    boost::lock_guard<boost::mutex> lock(set_mutex_);
    position_des_ = position;
    control_mode_.store(ControlMode::POSITION);
};

void HapticWristImpl::setOrientationGains(double kp, double kd) {
    boost::lock_guard<boost::mutex> lock(set_mutex_);
    orientation_controller_->setGains(kp, kd);
}

void HapticWristImpl::hold(bool hold) {
    boost::lock_guard<boost::mutex> lock(set_mutex_);
    if (hold) {
        // Capture the current orientation as the setpoint
        position_des_ = getPosition();
        control_mode_.store(ControlMode::POSITION);
    } else {
        // Release control
        control_mode_.store(ControlMode::NONE);
    }
}

void HapticWristImpl::setWristToBase(const Eigen::Matrix4d& transform) {
    boost::lock_guard<boost::mutex> lock(set_mutex_);
    base_to_wrist_ = transform;
}

void HapticWristImpl::gravityCompensate(bool compensate) {
    gravity_compensate_.store(compensate);
}

void HapticWristImpl::moveTo(const jp_type& desiredPos, double vel, double accel) {
    jp_type startPos = getPosition();
    Trajectory<jp_type> trajectory(startPos, desiredPos, vel, accel); 

    auto start_time = std::chrono::steady_clock::now();
    double elapsed_seconds = 0.0;
    double duration = trajectory.get_duration();
    while (elapsed_seconds < duration) {
        auto elapsed_time = std::chrono::steady_clock::now() - start_time;
        elapsed_seconds = std::chrono::duration<double>(elapsed_time).count();
        jp_type target = trajectory.get_setpoint_at(elapsed_seconds);
        setTarget(target);
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
}

void HapticWristImpl::moveTo(const Eigen::Quaterniond& desiredOrientation, double vel, double accel) {
    Eigen::Quaterniond startOrientation = getOrientation();
    Trajectory<Eigen::Quaterniond> trajectory(startOrientation, desiredOrientation, vel, accel); 

    auto start_time = std::chrono::steady_clock::now();
    double elapsed_seconds = 0.0;
    double duration = trajectory.get_duration();
    while (elapsed_seconds < duration) {
        auto elapsed_time = std::chrono::steady_clock::now() - start_time;
        elapsed_seconds = std::chrono::duration<double>(elapsed_time).count();
        Eigen::Quaterniond target = trajectory.get_setpoint_at(elapsed_seconds);
        setTarget(target);
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
}

void HapticWristImpl::run() {
    if (!running_.load()) {
        running_.store(true);
        control_thread_ = std::thread(&HapticWristImpl::entryPoint, this);
    }
}

void HapticWristImpl::stop() {
    running_.store(false);
    if (control_thread_.joinable()) {
        control_thread_.join();
    }
}

// --- Main Control Loop ---
bool HapticWristImpl::entryPoint() {
    while (running_.load()) {

        const auto loop_start_time = std::chrono::steady_clock::now();
        jt_type total_joint_torques = jt_type::Zero();
        
        // --- State Snapshot ---
        jp_type current_pos;
        jv_type current_vel;
        Eigen::Quaterniond current_orientation;
        {
            boost::shared_lock<boost::shared_mutex> lock(state_mutex_);
            current_pos = handle_theta_;
            current_vel = handle_dtheta_;
            current_orientation = handle_orientation_;
        }

        // --- Control Law Calculation ---
        ControlMode current_mode = control_mode_.load();
        if (current_mode == ControlMode::POSITION) {
            jp_type local_desired_position;
            {
                boost::lock_guard<boost::mutex> lock(set_mutex_);
                local_desired_position = position_des_;
            }

            jt_type joint_position_torque = joint_position_controller_->compute_torque(
                local_desired_position, current_pos);

            total_joint_torques += joint_position_torque;

        } else if (current_mode == ControlMode::ORIENTATION) {
            Eigen::Quaterniond desired_orientation;
            {
                boost::lock_guard<boost::mutex> lock(set_mutex_);
                desired_orientation = orientation_des_;
            }

            // 1. Calculate the Jacobian
            auto J_omega = kinematics_.jacobian_omega(current_pos);
            
            // 2. Calculate tool angular velocity in the base frame
            cv_type tool_vel_base_frame = J_omega * current_vel;

            // 3. Get Cartesian acceleration command from the orientation controller.
            // Note: We are re-interpreting the output of the PD controller as a desired
            //       acceleration, which is the standard formulation for operational space control.
            ct_type cartesian_accel_des = orientation_controller_->compute_torque(
                desired_orientation, current_orientation, tool_vel_base_frame);

            // 4. Map Cartesian acceleration to joint torques using Operational Space Control.
            // This is the key change to improve stability by accounting for joint inertia.

            // Define a simplified, diagonal joint-space inertia matrix M.
            // These values are weights representing the relative inertia of each joint.
            // Since you noted joint 3 is "lighter", we give it a smaller inertia value and match the new joint 4.
            // These values are tunable parameters for your specific hardware.
            Eigen::Matrix<double, kWristDofs, kWristDofs> M = Eigen::Matrix<double, kWristDofs, kWristDofs>::Identity();
            M(1, 1) = 0.6;
            M(2, 2) = 0.1;
            M(3, 3) = 0.1;

            Eigen::Matrix<double, kWristDofs, kWristDofs> M_inv = M.inverse(); // For a diagonal matrix, this is just 1/m_ii

            // Calculate the operational space inertia matrix, Lambda = (J * M^-1 * J^T)^-1
            Eigen::Matrix3d JM_invJT = J_omega * M_inv * J_omega.transpose();

            // Add a small amount of damping for numerical stability, especially near singularities.
            double damping = 1e-5;
            JM_invJT += Eigen::Matrix3d::Identity() * damping;

            Eigen::Matrix<double, 3, 3> Lambda = JM_invJT.inverse();

            // Calculate the Cartesian force required to achieve the desired acceleration
            ct_type cartesian_force_cmd = Lambda * cartesian_accel_des;

            // Map the dynamically-scaled Cartesian force to joint torques
            total_joint_torques += J_omega.transpose() * cartesian_force_cmd;
        }

        // --- Add Optional Gravity Compensation ---
        if (gravity_compensate_.load()) {
            Eigen::Matrix4d local_wrist_to_base;
            {
                boost::lock_guard<boost::mutex> lock(set_mutex_);
                local_wrist_to_base = base_to_wrist_;
            }
            auto kin = kinematics_.eval(current_pos, local_wrist_to_base);
            total_joint_torques += gravity_compensator_.eval(kin);
        }
        

        // --- Command Execution ---
        mt_type motor_torques = jtmp_matrix_ * total_joint_torques;
        if (executeControl(motor_torques)) {
            // A fault occurred, stop the loop
            running_.store(false);
        }


        const auto elapsed_time = std::chrono::steady_clock::now() - loop_start_time;
        const auto time_to_sleep = control_period_ - elapsed_time;

        // std::cout << "Total Time: " << std::chrono::duration_cast<std::chrono::microseconds>(elapsed_time).count() << " us" << std::endl;
        if (time_to_sleep > std::chrono::seconds::zero()) {
            std::this_thread::sleep_for(time_to_sleep);
        } else {
            // std::cerr << "Warning: Loop overrun detected! Consider lowering the control rate "
            //           << "Desired period: "
            //           << std::chrono::duration_cast<std::chrono::microseconds>(control_period_).count() << " us, "
            //           << "Actual time: "
            //           << std::chrono::duration_cast<std::chrono::microseconds>(elapsed_time).count() << " us"
            //           << std::endl;
        }
    }

    // On exit, brake the motors
    std::cout << "Control loop stopping. Engaging brake." << std::endl;
    for (auto& c : controllers_) {
        c->SetBrake();
    }
    return true;
}


bool HapticWristImpl::executeControl(const mt_type& des_motor_torque) {
    send_frames_.clear();
    for (size_t i = 0; i < controllers_.size(); i++) {
        cmd_.feedforward_torque = des_motor_torque(i);
        send_frames_.push_back(controllers_[i]->MakePosition(cmd_));
    }

    receive_frames_.clear();
    
    const auto can_start_time = std::chrono::steady_clock::now();
    transport_->BlockingCycle(&send_frames_[0], send_frames_.size(), &receive_frames_);
    const auto elapsed_time = std::chrono::steady_clock::now() - can_start_time;
    // std::cout << "Can period: " << std::chrono::duration_cast<std::chrono::microseconds>(elapsed_time).count() << " us" << std::endl;

    // --- Parse Responses and Update State ---
    auto maybe_servo1 = FindServo(receive_frames_, 1);
    auto maybe_servo2 = FindServo(receive_frames_, 2);
    auto maybe_servo3 = FindServo(receive_frames_, 3);
    auto maybe_servo4 = FindServo(receive_frames_, 4);

    if (!maybe_servo1 || !maybe_servo2 || !maybe_servo3 || !maybe_servo4) {
        missed_replies_++;
        if (missed_replies_ > 5) {
            std::cerr << "ERROR: Servos not responding. Halting." << std::endl;
            return true; // Return true for error
        }
        return false;
    } else {
        missed_replies_ = 0;
    }

    const auto& v1 = *maybe_servo1;
    const auto& v2 = *maybe_servo2;
    const auto& v3 = *maybe_servo3;
    const auto& v4 = *maybe_servo4;

    if (v1.mode == moteus::Mode::kFault || v2.mode == moteus::Mode::kFault || v3.mode == moteus::Mode::kFault ||
        v4.mode == moteus::Mode::kFault) {
        std::cerr << "ERROR: Servo fault detected. "
                  << "S1:" << v1.fault << " S2:" << v2.fault << " S3:" << v3.fault << " S4:" << v4.fault
                  << std::endl;
        return true; // Return true for error
    }

    mp_type motor_theta;
    motor_theta(0) = v1.position * radiansPerRotation;
    motor_theta(1) = v2.position * radiansPerRotation;
    motor_theta(2) = v3.position * radiansPerRotation;
    motor_theta(3) = v4.position * radiansPerRotation;

    mv_type motor_dtheta;
    motor_dtheta(0) = v1.velocity * radiansPerRotation;
    motor_dtheta(1) = v2.velocity * radiansPerRotation;
    motor_dtheta(2) = v3.velocity * radiansPerRotation;
    motor_dtheta(3) = v4.velocity * radiansPerRotation;

    mt_type motor_torque;
    motor_torque(0) = v1.torque;
    motor_torque(1) = v2.torque;
    motor_torque(2) = v3.torque;
    motor_torque(3) = v4.torque;
    
    // Lock and update the shared state variables
    {
        boost::unique_lock<boost::shared_mutex> lock(state_mutex_);
        handle_theta_ = compute_pos(motor_theta);
        handle_dtheta_ = compute_vel(motor_dtheta);
        handle_torque_ = compute_torque(motor_torque);

        // Update orientation from new joint positions
        auto kin = kinematics_.eval(handle_theta_);
        Eigen::Matrix3d rotation_matrix = kin.back().to_world_frame.block<3, 3>(0, 0);
        handle_orientation_ = Eigen::Quaterniond(rotation_matrix);
        handle_orientation_.normalize();
    }
    
    return false; // No error
}

// --- Getters and Helper Methods ---
jp_type HapticWristImpl::getHome() const {
    return home_;
}

jp_type HapticWristImpl::getPosition() {
    boost::shared_lock<boost::shared_mutex> lock(state_mutex_);
    return handle_theta_;
}
jv_type HapticWristImpl::getVelocity() {
    boost::shared_lock<boost::shared_mutex> lock(state_mutex_);
    return handle_dtheta_;
}
jt_type HapticWristImpl::getTorque() {
    boost::shared_lock<boost::shared_mutex> lock(state_mutex_);
    return handle_torque_;
}

const Kinematics& HapticWristImpl::getKinematics() const {
    return kinematics_;
}

Eigen::Quaterniond HapticWristImpl::getOrientation() {
    boost::shared_lock<boost::shared_mutex> lock(state_mutex_);
    return handle_orientation_;
}
jp_type HapticWristImpl::compute_pos(const mp_type& motor_theta) {
    return mtjp_matrix_ * motor_theta;
}
jv_type HapticWristImpl::compute_vel(const mv_type& motor_dtheta) {
    return mtjp_matrix_ * motor_dtheta;
}
jt_type HapticWristImpl::compute_torque(const mt_type& motor_torque) {
    return mtjp_matrix_ * motor_torque;
}
boost::optional<moteus::Query::Result>
HapticWristImpl::FindServo(const std::vector<moteus::CanFdFrame>& frames, int id) {
    for (auto it = frames.rbegin(); it != frames.rend(); ++it) {
        if (it->source == id) {
            return moteus::Query::Parse(it->data, it->size);
        }
    }
    return {};
}

} // namespace haptic_wrist
