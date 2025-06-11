#include <stdio.h>

#include "haptic_wrist/trapezoidal_velocity_profile.h"
#include "haptic_wrist_impl.h"

#include "utils.h"
#include "yaml-cpp/yaml.h"
#include <boost/optional.hpp>
#include <unistd.h>

// Updated joint limits for Z-Y-Z serial wrist
const double J1_MIN_THETA = -M_PI;      // Z-axis: ±180°
const double J1_MAX_THETA = M_PI;

const double J2_MIN_THETA = -M_PI;  // Y-axis: ±90°
const double J2_MAX_THETA = M_PI;

const double J3_MIN_THETA = -M_PI;      // Z-axis: ±180°
const double J3_MAX_THETA = M_PI;

using namespace mjbots;

namespace haptic_wrist {

HapticWristImpl::HapticWristImpl()
    : handle_theta(Eigen::Vector3d::Zero())
    , handle_dtheta(Eigen::Vector3d::Zero())
    , handle_torque(Eigen::Vector3d::Zero())
    , gravity(false) {

    j_pos_min(0) = J1_MIN_THETA;
    j_pos_min(1) = J2_MIN_THETA;
    j_pos_min(2) = J3_MIN_THETA;

    j_pos_max(0) = J1_MAX_THETA;
    j_pos_max(1) = J2_MAX_THETA;
    j_pos_max(2) = J3_MAX_THETA;

    // Diagonal transformation matrices with gear ratios
    jtmp_matrix = Eigen::Matrix3d::Zero();
    jtmp_matrix(0, 0) = 1.0 / MOTOR_TO_JOINT_GEAR_RATIO_1;  // Motor 1 to Joint 1
    jtmp_matrix(1, 1) = 1.0 / MOTOR_TO_JOINT_GEAR_RATIO_2;  // Motor 2 to Joint 2
    jtmp_matrix(2, 2) = 1.0 / MOTOR_TO_JOINT_GEAR_RATIO_3;  // Motor 3 to Joint 3

    // Inverse transformation
    mtjp_matrix = Eigen::Matrix3d::Zero();
    mtjp_matrix(0, 0) = MOTOR_TO_JOINT_GEAR_RATIO_1;  // Joint 1 to Motor 1
    mtjp_matrix(1, 1) = MOTOR_TO_JOINT_GEAR_RATIO_2;  // Joint 2 to Motor 2
    mtjp_matrix(2, 2) = MOTOR_TO_JOINT_GEAR_RATIO_3;  // Joint 3 to Motor 3

    std::string config_dir = get_config_directory();
    if (config_dir.empty()) {
        throw std::runtime_error("No valid configuration directory found.");
    }
    boost::filesystem::path config_file = boost::filesystem::path(config_dir) / "haptic_wrist.yaml";
    YAML::Node yaml_config = YAML::LoadFile(config_file.string());
    std::vector<DHParameter> dh;
    for (size_t i = 0; i < 3; i++) {
        DHParameter dh_param;
        dh_param.alpha_pi = yaml_config["kinematics"]["dh"][i]["alpha_pi"].as<double>();
        dh_param.a = yaml_config["kinematics"]["dh"][i]["a"].as<double>();
        dh_param.d = yaml_config["kinematics"]["dh"][i]["d"].as<double>();
        dh.push_back(dh_param);
    }
    kinematics = Kinematics(dh, Eigen::Matrix4d::Identity());
    config_file = boost::filesystem::path(config_dir) / "gravity_cal.yaml";
    YAML::Node mu_config = YAML::LoadFile(config_file.string());
    Eigen::Matrix3d mus;
    for (size_t row = 0; row < 3; row++) {
        for (size_t col = 0; col < 3; col++) {
            mus(row, col) = mu_config["mus"][row][col].as<double>();
        }
    }
    gravity_compensator = GravityComp(mus);

    moteus::Controller::Options options_common;

    auto& pf = options_common.position_format;
    pf.position = moteus::kFloat;
    pf.velocity = moteus::kFloat;
    pf.kp_scale = moteus::kFloat;
    pf.kd_scale = moteus::kFloat;
    pf.feedforward_torque = moteus::kFloat;

    transport = moteus::Controller::MakeSingletonTransport({});

    controllers = {
        std::make_shared<moteus::Controller>([&]() {
            auto options = options_common;
            options.id = 1;
            return options;
        }()),
        std::make_shared<moteus::Controller>([&]() {
            auto options = options_common;
            options.id = 2;
            return options;
        }()),
        std::make_shared<moteus::Controller>([&]() {
            auto options = options_common;
            options.id = 3;
            return options;
        }()),
    };

    for (auto& c : controllers) {
        c->SetStop();
    }

    cmd.position = 0.0;
    cmd.velocity = 0.0;
    cmd.kp_scale = 1.0;  // Use default moteus gains
    cmd.kd_scale = 1.0;  // Use default moteus gains
    cmd.feedforward_torque = 0.0;
};

HapticWristImpl::~HapticWristImpl() {
    stop();
}

void HapticWristImpl::setPosition(const jp_type& pos) {
    boost::lock_guard<boost::mutex> lock(set_mutex);
    Eigen::Vector3d jp_limited;
    // Limit joint positions
    for (int i = 0; i < 3; i++) {
        // jp_limited(i) = std::min(std::max(pos(i), j_pos_min(i)), j_pos_max(i));
        jp_limited(i) = pos(i);
    }

    theta_des = jtmp_matrix * jp_limited;
    has_setpoint.store(true);
};

void HapticWristImpl::setWristToBase(const Eigen::Matrix4d& transform) {
    boost::lock_guard<boost::mutex> lock(set_mutex);
    baseToWrist = transform;
}

jp_type HapticWristImpl::compute_pos(const mp_type& motor_theta) {
    return mtjp_matrix * motor_theta;
}

jv_type HapticWristImpl::compute_vel(const mv_type& motor_dtheta) {
    return mtjp_matrix * motor_dtheta;
}

jt_type HapticWristImpl::compute_torque(const mt_type& motor_torque) {
    return mtjp_matrix * motor_torque;
}

boost::optional<mjbots::moteus::Query::Result>
HapticWristImpl::FindServo(const std::vector<mjbots::moteus::CanFdFrame>& frames, int id) {
    for (auto it = frames.rbegin(); it != frames.rend(); ++it) {
        if (it->source == id) {
            return mjbots::moteus::Query::Parse(it->data, it->size);
        }
    }
    return {};
}

jp_type HapticWristImpl::getPosition() {
    boost::shared_lock<boost::shared_mutex> lock(state_mutex);
    return handle_theta;
}

jv_type HapticWristImpl::getVelocity() {
    boost::shared_lock<boost::shared_mutex> lock(state_mutex);
    return handle_dtheta;
}

jt_type HapticWristImpl::getTorque() {
    boost::shared_lock<boost::shared_mutex> lock(state_mutex);
    return handle_torque;
}

void HapticWristImpl::moveTo(const jp_type& desiredPos, double vel, double accel) {
    jp_type startPos = getPosition();
    jp_type diff = desiredPos - startPos;
    double distance = diff.norm();  // Use norm() instead of squaredNorm() for actual distance

    TrapezoidalVelocityProfile profile(vel, accel, 0.0, distance);
    auto start_time = std::chrono::steady_clock::now();
    double elapsed_seconds = 0.0;
    while (elapsed_seconds < profile.finalT()) {
        auto elapsed_time = std::chrono::steady_clock::now() - start_time;
        elapsed_seconds = std::chrono::duration<double>(elapsed_time).count();
        double arc_pos = profile.eval(elapsed_seconds);
        double norm_arc_pos = arc_pos / distance;
        jp_type goal_pos = (1.0 - norm_arc_pos) * startPos + norm_arc_pos * desiredPos;
        setPosition(goal_pos);
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    // Ensure we reach the final position
    setPosition(desiredPos);
}

void HapticWristImpl::gravityCompensate(bool compensate) {
    gravity = compensate;
}

void HapticWristImpl::run() {
    if (!running) {
        running = true;
        control_thread = std::thread(&HapticWristImpl::entryPoint, this);
    }
}

void HapticWristImpl::stop() {
    running.store(false);
    if (control_thread.joinable()) {
        control_thread.join();
    }
}

void HapticWristImpl::hold(bool hold) {
    if (hold) {
        setPosition(getPosition());
    } else {
        has_setpoint.store(false);
    }
}

bool HapticWristImpl::entryPoint() {
    mt_type feedforward_torque;
    while (running.load()) {
        feedforward_torque = Eigen::Vector3d::Zero();
        Eigen::Vector3d local_theta_des;
        {
            boost::lock_guard<boost::mutex> lock(set_mutex);
            local_theta_des = theta_des;
        }

        // Add gravity compensation if enabled
        if (gravity) {
            Eigen::Matrix4d local_wrist_to_base;
            {
                boost::lock_guard<boost::mutex> lock(set_mutex);
                local_wrist_to_base = baseToWrist;
            }
            std::array<Kin, 3> kin = kinematics.eval(handle_theta, local_wrist_to_base);
            auto g_torque = gravity_compensator.eval(kin);
            feedforward_torque += jtmp_matrix * g_torque;
        }
        
        bool status = executeControl(local_theta_des / (2.0 * M_PI), feedforward_torque);
        if (status) {
            running.store(false);
        }

        std::this_thread::sleep_for(std::chrono::microseconds(10));
    }

    printf("Entering fault mode!\n");

    for (auto& c : controllers) {
        c->SetBrake();
    }
    return true;
}

bool HapticWristImpl::executeControl(const mt_type& des_motor_pos, const mt_type& des_motor_torque) {

    send_frames.clear();

    for (size_t i = 0; i < controllers.size(); i++) {
        if (has_setpoint.load()) {
            cmd.position = des_motor_pos(i);
            cmd.kp_scale = 1.0;  // Use default moteus position gains
            cmd.kd_scale = 1.0;  // Use default moteus velocity gains
        } else {
            cmd.position = std::numeric_limits<double>::quiet_NaN();  // No position control
            cmd.kp_scale = 0.0;  // Disable position control
            cmd.kd_scale = 0.1;  // Keep some damping
        }
        cmd.velocity = 0.0;
        cmd.feedforward_torque = des_motor_torque(i);
        send_frames.push_back(controllers[i]->MakePosition(cmd));
    }

    receive_frames.clear();
    transport->BlockingCycle(&send_frames[0], send_frames.size(), &receive_frames);

    auto maybe_servo1 = FindServo(receive_frames, 1);
    auto maybe_servo2 = FindServo(receive_frames, 2);
    auto maybe_servo3 = FindServo(receive_frames, 3);

    if (!maybe_servo1 || !maybe_servo2 || !maybe_servo3) {
        missed_replies++;
        if (missed_replies > 3) {
            printf("\n\nServo not responding 1=%d 2=%d 3=%d\n", maybe_servo1 ? 1 : 0, maybe_servo2 ? 1 : 0,
                   maybe_servo3 ? 1 : 0);
            return true;
        }
        return true;
    } else {
        missed_replies = 0;
    }

    const auto& v1 = *maybe_servo1;
    const auto& v2 = *maybe_servo2;
    const auto& v3 = *maybe_servo3;
    Eigen::Vector3d motor_theta;

    if (v1.mode == moteus::Mode::kFault) {
        std::cout << "Servo 1 fault" << std::endl;
        return true;
    }

    if (v2.mode == moteus::Mode::kFault) {
        std::cout << "Servo 2 fault" << std::endl;
        return true;
    }

    if (v3.mode == moteus::Mode::kFault) {
        std::cout << "Servo 3 fault" << std::endl;
        return true;
    }

    motor_theta(0) = v1.position * radiansPerRotation;
    motor_theta(1) = v2.position * radiansPerRotation;
    motor_theta(2) = v3.position * radiansPerRotation;

    Eigen::Vector3d motor_dtheta;
    motor_dtheta(0) = v1.velocity * radiansPerRotation;
    motor_dtheta(1) = v2.velocity * radiansPerRotation;
    motor_dtheta(2) = v3.velocity * radiansPerRotation;

    Eigen::Vector3d motor_torque;
    motor_torque(0) = v1.torque;
    motor_torque(1) = v2.torque;
    motor_torque(2) = v3.torque;

    {
        boost::unique_lock<boost::shared_mutex> lock(state_mutex);
        handle_theta = compute_pos(motor_theta);
        handle_dtheta = compute_vel(motor_dtheta);
        handle_torque = compute_torque(motor_torque);
    }
    return 0;
}

} // namespace haptic_wrist
