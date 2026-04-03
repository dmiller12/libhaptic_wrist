#include "haptic_wrist_impl.h"
#include "haptic_wrist/haptic_wrist_config.h"
#include "utils.h"
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <boost/optional.hpp>

#include "config_loader.h"
#include "trajectory.h"

using namespace mjbots;

namespace haptic_wrist {

namespace {

double WrapToPi(double angle) {
    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
}

} // namespace

HapticWristImpl::HapticWristImpl()
    : handle_theta_(Eigen::Vector2d::Zero())
    , handle_dtheta_(Eigen::Vector2d::Zero())
    , handle_torque_(Eigen::Vector2d::Zero())
    , handle_kin_theta_(Eigen::Vector3d::Zero())
    , handle_kin_dtheta_(Eigen::Vector3d::Zero())
    , handle_orientation_(Eigen::Quaterniond::Identity())
    , control_period_(1.0 / control_rate_) {

    const std::string config_dir = get_config_directory();
    if (config_dir.empty()) {
        throw std::runtime_error("No valid configuration directory found.");
    }

    const HapticWristConfig config = load_config(config_dir);


    home_ = config.home_position;
    handle_kin_theta_ << 0.0, home_(0), home_(1);
    passive_offset_rad_ = config.passive_encoder.offset_rad;
    passive_scale_ = config.passive_encoder.scale;

    kinematics_ = Kinematics(config.dh_parameters, config.eef_to_tool, Eigen::Matrix4d::Identity());
    gravity_compensator_ = GravityComp(config.gravity_mus);

    joint_position_controller_ = std::make_unique<JointPositionController>(
        config.joint_position_controller.kp, config.joint_position_controller.kd, control_period_.count());

    // Transformation matrices for motor/joint conversions.
    jtmp_matrix_ = config.j2mp;
    mtjp_matrix_ = jtmp_matrix_.inverse();


    // Configure moteus controllers for torque control.
    moteus::Controller::Options options_common;
    auto& pf = options_common.position_format;
    pf.position = moteus::kIgnore;
    pf.velocity = moteus::kIgnore;
    pf.feedforward_torque = moteus::kFloat;
    pf.kd_scale = moteus::kIgnore;
    pf.kp_scale = moteus::kIgnore;

    auto& qf = options_common.query_format;
    qf.voltage = moteus::kIgnore;
    qf.temperature = moteus::kIgnore;
    // Passive DoF is read from motor_position.sources.1 on controller ID 1.
    qf.extra[0].register_number = moteus::Register::kEncoder1Position;
    qf.extra[0].resolution = moteus::kFloat;
    qf.extra[1].register_number = moteus::Register::kEncoder1Velocity;
    qf.extra[1].resolution = moteus::kFloat;
    qf.extra[2].register_number = moteus::Register::kEncoderValidity;
    qf.extra[2].resolution = moteus::kInt8;


    // This sets up the global transport singleton according to configured args.
    // moteus::Controller::ProcessTransportArgs(config.moteus.transport_args);
    // transport_ = moteus::Controller::MakeSingletonTransport({});
    // cant use singleton if 2 usb's are plugged in at the same time
    transport_ = std::make_shared<moteus::Fdcanusb>(config.moteus.transport_args);

    // Active controllers:
    // ID 1 -> WAM J5 (also hosts AUX2 passive encoder)
    // ID 2 -> WAM J6
    controllers_ = {
        std::make_shared<moteus::Controller>([&]() {
            auto opts = options_common;
            opts.id = 1;
            return opts;
        }()),
        std::make_shared<moteus::Controller>([&]() {
            auto opts = options_common;
            opts.id = 2;
            return opts;
        }()),
    };


    // Set moteus params and initialize motors to a stopped state.
    size_t i = 0;
    for (auto& c : controllers_) {
        c->DiagnosticWrite("tel stop\n");
        c->DiagnosticFlush();
        c->DiagnosticCommand("conf set servo.pid_position.kp 0");
        c->DiagnosticCommand("conf set servo.pid_position.ki 0");
        std::ostringstream kd_cmd;
        kd_cmd << "conf set servo.pid_position.kd " << config.moteus.kd(i);
        c->DiagnosticCommand(kd_cmd.str());
        c->SetStop();
        ++i;
    }

}

HapticWristImpl::~HapticWristImpl() {
    stop();
}

void HapticWristImpl::setTarget(const Eigen::Quaterniond& orientation) {
    (void)orientation;
    throw std::logic_error("Orientation target is disabled. Use joint setTarget(jp_type) for joint-to-joint control.");
}

void HapticWristImpl::setTarget(const jp_type& position) {
    boost::lock_guard<boost::mutex> lock(set_mutex_);
    position_des_ = position;
    control_mode_.store(ControlMode::POSITION);
}

void HapticWristImpl::setOrientationGains(double kp, double kd) {
    (void)kp;
    (void)kd;
    throw std::logic_error("Orientation control is disabled in joint-to-joint mode.");
}

void HapticWristImpl::hold(bool hold) {
    boost::lock_guard<boost::mutex> lock(set_mutex_);
    if (hold) {
        // Capture the current active joint positions as the setpoint.
        position_des_ = getPosition();
        control_mode_.store(ControlMode::POSITION);
    } else {
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

void HapticWristImpl::jointMoveTo(const jp_type& desiredPos, double vel, double accel) {
    const jp_type startPos = getPosition();
    Trajectory<jp_type> trajectory(startPos, desiredPos, vel, accel);

    const auto start_time = std::chrono::steady_clock::now();
    double elapsed_seconds = 0.0;
    const double duration = trajectory.get_duration();
    while (elapsed_seconds < duration) {
        const auto elapsed_time = std::chrono::steady_clock::now() - start_time;
        elapsed_seconds = std::chrono::duration<double>(elapsed_time).count();
        const jp_type target = trajectory.get_setpoint_at(elapsed_seconds);
        setTarget(target);
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
}

void HapticWristImpl::moveTo(const Eigen::Quaterniond& desiredOrientation, double vel, double accel) {
    (void)desiredOrientation;
    (void)vel;
    (void)accel;
    throw std::logic_error("Orientation move is disabled. Use jointMoveTo(jp_type) for joint-to-joint control.");
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
        kq_type current_kin_pos;
        {
            boost::shared_lock<boost::shared_mutex> lock(state_mutex_);
            current_pos = handle_theta_;
            current_kin_pos = handle_kin_theta_;
        }

        // --- Control Law Calculation ---
        const ControlMode current_mode = control_mode_.load();
        if (current_mode == ControlMode::POSITION) {
            jp_type local_desired_position;
            {
                boost::lock_guard<boost::mutex> lock(set_mutex_);
                local_desired_position = position_des_;
            }

            const jt_type joint_position_torque =
                joint_position_controller_->compute_torque(local_desired_position, current_pos);
            total_joint_torques += joint_position_torque;
        }

        // --- Add Optional Gravity Compensation ---
        if (gravity_compensate_.load()) {
            Eigen::Matrix4d local_wrist_to_base;
            {
                boost::lock_guard<boost::mutex> lock(set_mutex_);
                local_wrist_to_base = base_to_wrist_;
            }

            const std::array<Kin, 4> full_kin = kinematics_.eval(current_kin_pos, local_wrist_to_base);
            const std::array<Kin, 3> active_kin = {full_kin[1], full_kin[2], full_kin[3]};
            total_joint_torques += gravity_compensator_.eval(active_kin);
        }

        // --- Command Execution ---
        const mt_type motor_torques = jtmp_matrix_ * total_joint_torques;
        if (executeControl(motor_torques)) {
            running_.store(false);
        }

        const auto elapsed_time = std::chrono::steady_clock::now() - loop_start_time;
        const auto time_to_sleep = control_period_ - elapsed_time;
        if (time_to_sleep > std::chrono::seconds::zero()) {
            std::this_thread::sleep_for(time_to_sleep);
        }
    }

    // On exit, brake the motors.
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
    transport_->BlockingCycle(&send_frames_[0], send_frames_.size(), &receive_frames_);

    // --- Parse Responses and Update State ---
    const auto maybe_servo1 = FindServo(receive_frames_, 1);
    const auto maybe_servo2 = FindServo(receive_frames_, 2);

    if (!maybe_servo1 || !maybe_servo2) {
        missed_replies_++;
        if (missed_replies_ > 5) {
            std::cerr << "ERROR: Servos not responding. Halting." << std::endl;
            return true;
        }
        return false;
    }

    missed_replies_ = 0;
    const auto& v1 = *maybe_servo1;
    const auto& v2 = *maybe_servo2;

    if (v1.mode == moteus::Mode::kFault || v2.mode == moteus::Mode::kFault) {
        std::cerr << "ERROR: Servo fault detected. "
                  << "S1:" << v1.fault << " S2:" << v2.fault << std::endl;
        return true;
    }

    // Active motor state from controller outputs.
    mp_type motor_theta;
    motor_theta(0) = v1.position * radiansPerRotation;
    motor_theta(1) = v2.position * radiansPerRotation;

    mv_type motor_dtheta;
    motor_dtheta(0) = v1.velocity * radiansPerRotation;
    motor_dtheta(1) = v2.velocity * radiansPerRotation;

    mt_type motor_torque;
    motor_torque(0) = v1.torque;
    motor_torque(1) = v2.torque;

    // Passive state from encoder slot 1 (motor_position.sources.1) on controller ID 1.
    const double passive_pos_turns = FindExtraRegister(v1, moteus::Register::kEncoder1Position);
    const double passive_vel_turns = FindExtraRegister(v1, moteus::Register::kEncoder1Velocity);
    const double encoder_validity = FindExtraRegister(v1, moteus::Register::kEncoderValidity);

    double passive_pos_rad = 0.0;
    double passive_vel_rad_s = 0.0;
    {
        boost::shared_lock<boost::shared_mutex> lock(state_mutex_);
        passive_pos_rad = handle_kin_theta_(0);
        passive_vel_rad_s = handle_kin_dtheta_(0);
    }
    const double last_passive_pos_rad = passive_pos_rad;
    const double last_passive_vel_rad_s = passive_vel_rad_s;

    bool have_new_passive_pos = false;
    bool have_new_passive_vel = false;

    if (std::isfinite(passive_pos_turns)) {
        passive_pos_rad = passive_pos_turns * radiansPerRotation;
        have_new_passive_pos = true;
    }
    if (std::isfinite(passive_vel_turns)) {
        passive_vel_rad_s = passive_vel_turns * radiansPerRotation;
        have_new_passive_vel = true;
    }

    if (std::isfinite(encoder_validity)) {
        const int validity = static_cast<int>(std::llround(encoder_validity));
        const bool encoder1_valid = (validity & (1 << 1)) != 0;
        if (!encoder1_valid) {
            // Keep last good passive reading if slot 1 is not currently valid.
            passive_pos_rad = last_passive_pos_rad;
            passive_vel_rad_s = last_passive_vel_rad_s;
            have_new_passive_pos = false;
            have_new_passive_vel = false;
        }
    }

    if (have_new_passive_pos) {
        passive_pos_rad = WrapToPi((passive_pos_rad - passive_offset_rad_) * passive_scale_);
    }
    if (have_new_passive_vel) {
        passive_vel_rad_s *= passive_scale_;
    }

    {
        boost::unique_lock<boost::shared_mutex> lock(state_mutex_);
        handle_theta_ = compute_pos(motor_theta);
        handle_dtheta_ = compute_vel(motor_dtheta);
        handle_torque_ = compute_torque(motor_torque);

        handle_kin_theta_ << passive_pos_rad, handle_theta_(0), handle_theta_(1);
        handle_kin_dtheta_ << passive_vel_rad_s, handle_dtheta_(0), handle_dtheta_(1);

        const std::array<Kin, 4> kin = kinematics_.eval(handle_kin_theta_);
        const Eigen::Matrix3d rotation_matrix = kin[3].to_world_frame.block<3, 3>(0, 0);
        handle_orientation_ = Eigen::Quaterniond(rotation_matrix);
        handle_orientation_.normalize();
    }

    return false;
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

boost::optional<handle_type> HapticWristImpl::getHandle() {
    boost::shared_lock<boost::shared_mutex> lock(state_mutex_);

    using FrameType = decltype(receive_frames_)::value_type; 
    
    FrameType tx_frame;
    tx_frame.destination = 0x22;
    tx_frame.size = 1;
    tx_frame.data[0] = current_stiffness_;

    std::vector<FrameType> send_frame;
    send_frame.push_back(tx_frame);
    
    receive_frames_.clear();
    transport_->BlockingCycle(send_frame.data(), send_frame.size(), &receive_frames_);

    const int center_x = 785;
    const int center_y = 800;
    const int deadzone = 40;
    const int trigger_max_pos = 203;
    const int trigger_min_pos = 45;

    for (const auto& rx : receive_frames_) {
        int dest = static_cast<int>(rx.destination);
        int size = static_cast<int>(rx.size);

        // Moteus maps a standard CAN ID of 0x11 to the destination field
        if (dest == 0x11 && size >= 7) {
            int raw_trigger = (rx.data[0] << 8) | rx.data[1];
            int raw_thumbX  = (rx.data[2] << 8) | rx.data[3];
            int raw_thumbY  = (rx.data[4] << 8) | rx.data[5];
            int raw_bumper  = rx.data[6];
            int spring_force  = rx.data[7];


            // map the joysticks into the -1->1 range. Include deadzone so you dont do things with a little jitter
            // the resting pos of the joystick is not the halfway point of the values so we have to do this if check
            double f_thumbX = 0.0;
            if (raw_thumbX < (center_x - deadzone)) {
                f_thumbX = static_cast<double>(raw_thumbX - center_x) / center_x;
            } else if (raw_thumbX > (center_x + deadzone)) {
                f_thumbX = static_cast<double>(raw_thumbX - center_x) / (1023.0 - center_x);
            }

            double f_thumbY = 0.0;
            if (raw_thumbY < (center_y - deadzone)) {
                f_thumbY = static_cast<double>(raw_thumbY - center_y) / center_y;
            } else if (raw_thumbY > (center_y + deadzone)) {
                f_thumbY = static_cast<double>(raw_thumbY - center_y) / (1023.0 - center_y);
            }
            
            // map trigger from 0->1 . 0 is not pressed and 1 is fully pressed
            double f_trigger = static_cast<double>(raw_trigger - trigger_min_pos) / (trigger_max_pos - trigger_min_pos);
            f_trigger = std::max(0.0, std::min(f_trigger, 1.0)); 
            f_trigger = 1 - f_trigger;
            
            double f_bumper = static_cast<double>(raw_bumper);

            return handle_type(f_thumbX, f_thumbY, f_bumper, f_trigger);
        }
    }

    return boost::none;
}

// stiffness is between 0 - 255
void HapticWristImpl::setTriggerHaptics(uint8_t stiffness) {
    boost::unique_lock<boost::shared_mutex> lock(state_mutex_);
    current_stiffness_ = stiffness;
}

double HapticWristImpl::getPassivePosition() {
    boost::shared_lock<boost::shared_mutex> lock(state_mutex_);
    return handle_kin_theta_(0);
}

double HapticWristImpl::getPassiveVelocity() {
    boost::shared_lock<boost::shared_mutex> lock(state_mutex_);
    return handle_kin_dtheta_(0);
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

double HapticWristImpl::FindExtraRegister(const moteus::Query::Result& result, int16_t register_number) {
    for (const auto& extra_value : result.extra) {
        if (extra_value.register_number == std::numeric_limits<int16_t>::max()) {
            break;
        }
        if (extra_value.register_number == register_number) {
            return extra_value.value;
        }
    }
    return std::numeric_limits<double>::quiet_NaN();
}

} // namespace haptic_wrist
