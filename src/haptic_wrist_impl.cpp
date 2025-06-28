#include "haptic_wrist_impl.h"
#include "utils.h"
#include "yaml-cpp/yaml.h"
#include <boost/filesystem.hpp>
#include <iostream>

using namespace mjbots;

namespace haptic_wrist {

HapticWristImpl::HapticWristImpl()
    : handle_theta_(Eigen::Vector3d::Zero())
    , handle_dtheta_(Eigen::Vector3d::Zero())
    , handle_torque_(Eigen::Vector3d::Zero())
    , handle_orientation_(Eigen::Quaterniond::Identity())
    , orientation_des_(Eigen::Quaterniond::Identity()) {

    // Initialize the orientation controller with default gains.
    // These should be tuned for your specific hardware.
    orientation_controller_ = std::make_unique<OrientationController>(20.0, 0.5); // Kp=20, Kd=0.5

    // Transformation matrices for motor/joint conversions
    jtmp_matrix_ = Eigen::Matrix3d::Zero();
    jtmp_matrix_(0, 0) = 1.0 / MOTOR_TO_JOINT_GEAR_RATIO_1;
    jtmp_matrix_(1, 1) = 1.0 / MOTOR_TO_JOINT_GEAR_RATIO_2;
    jtmp_matrix_(2, 2) = 1.0 / MOTOR_TO_JOINT_GEAR_RATIO_3;

    mtjp_matrix_ = jtmp_matrix_.inverse();

    // Load DH parameters and gravity compensation data from YAML files
    std::string config_dir = get_config_directory();
    if (config_dir.empty()) {
        throw std::runtime_error("No valid configuration directory found.");
    }

    try {
        boost::filesystem::path config_file = boost::filesystem::path(config_dir) / "haptic_wrist.yaml";
        YAML::Node yaml_config = YAML::LoadFile(config_file.string());
        std::vector<DHParameter> dh;
        for (size_t i = 0; i < 3; i++) {
            // BUGFIX: Fully populate the DHParameter struct, including the theta_pi offset.
            DHParameter p;
            p.alpha_pi = yaml_config["kinematics"]["dh"][i]["alpha_pi"].as<double>();
            p.a = yaml_config["kinematics"]["dh"][i]["a"].as<double>();
            p.d = yaml_config["kinematics"]["dh"][i]["d"].as<double>();
            
            // Safely load theta_pi, as it may not exist for all joints.
            if (yaml_config["kinematics"]["dh"][i]["theta_pi"]) {
                p.theta_pi = yaml_config["kinematics"]["dh"][i]["theta_pi"].as<double>();
            } else {
                p.theta_pi = 0.0;
            }
            dh.push_back(p);
        }
        kinematics_ = Kinematics(dh, Eigen::Matrix4d::Identity());

        config_file = boost::filesystem::path(config_dir) / "gravity_cal.yaml";
        YAML::Node mu_config = YAML::LoadFile(config_file.string());
        Eigen::Matrix3d mus;
        for (size_t row = 0; row < 3; row++) {
            for (size_t col = 0; col < 3; col++) {
                mus(row, col) = mu_config["mus"][row][col].as<double>();
            }
        }
        gravity_compensator_ = GravityComp(mus);
    } catch (const YAML::Exception& e) {
        std::cerr << "Error loading configuration file: " << e.what() << std::endl;
        throw;
    }

    // Configure Moteus controllers for TORQUE control
    moteus::Controller::Options options_common;
    auto& pf = options_common.position_format;
    pf.position = moteus::kIgnore;
    pf.velocity = moteus::kInt8;
    pf.feedforward_torque = moteus::kFloat;
    pf.kp_scale = moteus::kInt8; // We will send 0 for these scales
    pf.kd_scale = moteus::kInt8;
    
    transport_ = moteus::Controller::MakeSingletonTransport({});
    controllers_ = {
        std::make_shared<moteus::Controller>([&]() { auto opts = options_common; opts.id = 1; return opts; }()),
        std::make_shared<moteus::Controller>([&]() { auto opts = options_common; opts.id = 2; return opts; }()),
        std::make_shared<moteus::Controller>([&]() { auto opts = options_common; opts.id = 3; return opts; }())
    };

    // Initialize motors to a stopped state
    for (auto& c : controllers_) { c->SetStop(); }
};

HapticWristImpl::~HapticWristImpl() {
    stop();
}

void HapticWristImpl::setOrientation(const Eigen::Quaterniond& orientation) {
    boost::lock_guard<boost::mutex> lock(set_mutex_);
    orientation_des_ = orientation.normalized();
    control_mode_.store(ControlMode::ORIENTATION);
};

void HapticWristImpl::setOrientationGains(double kp, double kd) {
    boost::lock_guard<boost::mutex> lock(set_mutex_);
    orientation_controller_->setGains(kp, kd);
}

void HapticWristImpl::hold(bool hold) {
    boost::lock_guard<boost::mutex> lock(set_mutex_);
    if (hold) {
        // Capture the current orientation as the setpoint
        orientation_des_ = getOrientation(); 
        control_mode_.store(ControlMode::ORIENTATION);
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
        if (current_mode == ControlMode::ORIENTATION) {
            Eigen::Quaterniond desired_orientation;
            {
                boost::lock_guard<boost::mutex> lock(set_mutex_);
                desired_orientation = orientation_des_;
            }

            // 1. Calculate the Jacobian
            Eigen::Matrix<double, 3, 3> J_omega = kinematics_.jacobian_omega(current_pos);
            
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
            // Since you noted joint 3 is "lighter", we give it a smaller inertia value.
            // These values are tunable parameters for your specific hardware.
            Eigen::Matrix3d M;
            M << 1.0, 0.0, 0.0,
                 0.0, 0.6, 0.0,
                 0.0, 0.0, 0.1; // m3 is smaller than m1 and m2

            Eigen::Matrix3d M_inv = M.inverse(); // For a diagonal matrix, this is just 1/m_ii

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
            std::array<Kin, 3> kin = kinematics_.eval(current_pos, local_wrist_to_base);
            total_joint_torques += gravity_compensator_.eval(kin);
        }
        
        // --- Command Execution ---
        mt_type motor_torques = jtmp_matrix_ * total_joint_torques;
        if (executeControl(motor_torques)) {
            // A fault occurred, stop the loop
            running_.store(false);
        }

        std::this_thread::sleep_for(std::chrono::microseconds(1000)); // ~1kHz loop
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
        cmd_.kp_scale = 0.0;
        cmd_.kd_scale = 0.1;
        cmd_.velocity = 0.0;
        cmd_.feedforward_torque = des_motor_torque(i);
        send_frames_.push_back(controllers_[i]->MakePosition(cmd_));
    }

    receive_frames_.clear();
    transport_->BlockingCycle(&send_frames_[0], send_frames_.size(), &receive_frames_);

    // --- Parse Responses and Update State ---
    auto maybe_servo1 = FindServo(receive_frames_, 1);
    auto maybe_servo2 = FindServo(receive_frames_, 2);
    auto maybe_servo3 = FindServo(receive_frames_, 3);

    if (!maybe_servo1 || !maybe_servo2 || !maybe_servo3) {
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

    if (v1.mode == moteus::Mode::kFault || v2.mode == moteus::Mode::kFault || v3.mode == moteus::Mode::kFault) {
        std::cerr << "ERROR: Servo fault detected. " 
                  << "S1:" << v1.fault << " S2:" << v2.fault << " S3:" << v3.fault << std::endl;
        return true; // Return true for error
    }

    mp_type motor_theta;
    motor_theta(0) = v1.position * radiansPerRotation;
    motor_theta(1) = v2.position * radiansPerRotation;
    motor_theta(2) = v3.position * radiansPerRotation;

    mv_type motor_dtheta;
    motor_dtheta(0) = v1.velocity * radiansPerRotation;
    motor_dtheta(1) = v2.velocity * radiansPerRotation;
    motor_dtheta(2) = v3.velocity * radiansPerRotation;

    mt_type motor_torque;
    motor_torque(0) = v1.torque;
    motor_torque(1) = v2.torque;
    motor_torque(2) = v3.torque;
    
    // Lock and update the shared state variables
    {
        boost::unique_lock<boost::shared_mutex> lock(state_mutex_);
        handle_theta_ = compute_pos(motor_theta);
        handle_theta_[1] += M_PI/2;
        handle_dtheta_ = compute_vel(motor_dtheta);
        handle_torque_ = compute_torque(motor_torque);

        // Update orientation from new joint positions
        std::array<Kin, 3> kin = kinematics_.eval(handle_theta_);
        Eigen::Matrix3d rotation_matrix = kin[2].to_world_frame.block<3, 3>(0, 0);
        handle_orientation_ = Eigen::Quaterniond(rotation_matrix);
    }
    
    return false; // No error
}

// --- Getters and Helper Methods ---
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
