#include <stdio.h>

#include "haptic_wrist/haptic_wrist.h"
#include "haptic_wrist/trapezoidal_velocity_profile.h"

#include "yaml-cpp/yaml.h"
#include <boost/optional.hpp>
#include <unistd.h>

// TODO: confirm correct
const double J1_MIN_THETA = -1.5;
const double J1_MAX_THETA = 1.0;

const double J2_MIN_THETA = -M_PI / 2;
const double J2_MAX_THETA = M_PI / 2;

const double J3_MIN_THETA = -0.9;
const double J3_MAX_THETA = 0.9;

using namespace mjbots;

namespace haptic_wrist {

HapticWrist::HapticWrist() : gravity(false) {

    jtmp_matrix << 0, -MOTOR_TO_HANDLE_SCALE_FACTOR_M1, -MOTOR_TO_HANDLE_SCALE_FACTOR_M1, 0,
        MOTOR_TO_HANDLE_SCALE_FACTOR_M2, -MOTOR_TO_HANDLE_SCALE_FACTOR_M2, MOTOR_TO_HANDLE_SCALE_FACTOR_M3, 0, 0;

    mtjp_matrix << 0, 0, 1.0 / MOTOR_TO_HANDLE_SCALE_FACTOR_M3, -0.5 / MOTOR_TO_HANDLE_SCALE_FACTOR_M1,
        0.5 / MOTOR_TO_HANDLE_SCALE_FACTOR_M2, 0, -0.5 / MOTOR_TO_HANDLE_SCALE_FACTOR_M1,
        -0.5 / MOTOR_TO_HANDLE_SCALE_FACTOR_M2, 0;

    YAML::Node yaml_config = YAML::LoadFile("../config/haptic_wrist.yaml");
    std::vector<DHParameter> dh;
    for (size_t i = 0; i < 3; i++) {
        DHParameter dh_param;
        dh_param.alpha_pi = yaml_config["kinematics"]["dh"][i]["alpha_pi"].as<double>();
        dh_param.a = yaml_config["kinematics"]["dh"][i]["a"].as<double>();
        dh_param.d = yaml_config["kinematics"]["dh"][i]["d"].as<double>();
        dh.push_back(dh_param);
    }

    kinematics = Kinematics(dh, Eigen::Matrix4d::Identity());
    YAML::Node mu_config = YAML::LoadFile("../config/gravity_cal.yaml");
    Eigen::Matrix3d mus;
    for (size_t row = 0; row < 3; row++) {
        for (size_t col = 0; col < 3; col++) {
            mus(row, col) = mu_config["mus"][row][col].as<double>();
        }
    }
    gravity_compensator = GravityComp(mus);
    
    // set diagonal elements
    for (size_t i = 0; i < 3; i++) {
        kp_axis(i, i) = yaml_config["kp"][i].as<double>();
        kd_axis(i, i) = yaml_config["kd"][i].as<double>();
    }

    moteus::Controller::Options options_common;

    auto &pf = options_common.position_format;
    pf.position = moteus::kFloat;
    pf.velocity = moteus::kFloat;
    pf.kp_scale = moteus::kInt8;
    pf.kd_scale = moteus::kInt8;
    pf.feedforward_torque = moteus::kFloat;

    // would be nice to remove this or have an alternative
    // moteus::Controller::DefaultArgProcess(argc, argv);
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

    for (auto &c : controllers) {
        c->SetStop();
    }

    cmd.position = 0.0;
    cmd.velocity = 0.0;
    cmd.kp_scale = 1.0;
    cmd.kd_scale = 1.0;
    cmd.feedforward_torque = 0.0;
};

HapticWrist::~HapticWrist() {
    stop();
    // TODO: should also break motors here
}

void HapticWrist::set_position(const jp_type &pos) {
    boost::lock_guard<boost::mutex> lock(set_mutex);
    Eigen::Vector3d jp_limited;
    double j1_limited = std::min(std::max(pos(0), J1_MIN_THETA), J1_MAX_THETA);
    double j2_limited = std::min(std::max(pos(1), J2_MIN_THETA), J2_MAX_THETA);
    double j3_limited = std::min(std::max(pos(2), J3_MIN_THETA), J3_MAX_THETA);
    jp_limited(0) = j1_limited;
    jp_limited(1) = j2_limited;
    jp_limited(2) = j3_limited;

    theta_des = jtmp_matrix * jp_limited;
    has_setpoint.store(true);
};

void HapticWrist::set_wrist_to_base(const Eigen::Matrix4d &transform) {
    boost::lock_guard<boost::mutex> lock(set_mutex);
    baseToWrist = transform;
}

jp_type HapticWrist::compute_pos(const mp_type &motor_theta) { return mtjp_matrix * motor_theta; }

jv_type HapticWrist::compute_vel(const mv_type &motor_dtheta) { return mtjp_matrix * motor_dtheta; }

jt_type HapticWrist::compute_torque(const mt_type &motor_torque) { return mtjp_matrix * motor_torque; }

boost::optional<mjbots::moteus::Query::Result>
HapticWrist::FindServo(const std::vector<mjbots::moteus::CanFdFrame> &frames, int id) {
    for (auto it = frames.rbegin(); it != frames.rend(); ++it) {
        if (it->source == id) {
            return mjbots::moteus::Query::Parse(it->data, it->size);
        }
    }
    return {};
}

jp_type HapticWrist::get_position() {
    boost::shared_lock<boost::shared_mutex> lock(state_mutex);
    return handle_theta;
}

jv_type HapticWrist::get_velocity() {
    boost::shared_lock<boost::shared_mutex> lock(state_mutex);
    return handle_dtheta;
}

jt_type HapticWrist::get_torque() {
    boost::shared_lock<boost::shared_mutex> lock(state_mutex);
    return handle_torque;
}

void HapticWrist::moveTo(const jp_type &desiredPos, double vel, double accel) {
    jp_type startPos = get_position();
    jp_type diff = desiredPos - startPos;
    double distance = diff.squaredNorm();

    TrapezoidalVelocityProfile profile(vel, accel, 0.0, distance);
    auto start_time = std::chrono::steady_clock::now();
    double elapsed_seconds = 0.0;
    while (elapsed_seconds < profile.finalT()) {
        auto elapsed_time = std::chrono::steady_clock::now() - start_time;
        elapsed_seconds = std::chrono::duration<double>(elapsed_time).count();
        double arc_pos = profile.eval(elapsed_seconds);
        double norm_arc_pos = arc_pos / distance;
        jp_type goal_pos = (1.0 - norm_arc_pos) * startPos + norm_arc_pos * desiredPos;
        set_position(goal_pos);
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
}

void HapticWrist::gravity_compensate(bool compensate) { gravity = compensate; }

void HapticWrist::run() {
    if (!running) {
        running = true;
        control_thread = std::thread(&HapticWrist::entryPoint, this);
    }
}

void HapticWrist::stop() {
    running.store(false);
    if (control_thread.joinable()) {
        control_thread.join();
    }
}

void HapticWrist::hold(bool hold) {
    if (hold) {
        set_position(get_position());
    } else {
        has_setpoint.store(false);
    }
}

bool HapticWrist::entryPoint() {
    Eigen::Vector3d prevError = Eigen::Vector3d::Zero();
    std::chrono::steady_clock::time_point last_time = std::chrono::steady_clock::now();
    mt_type feedforward_torque;
    while (running.load()) {
        feedforward_torque = Eigen::Vector3d::Zero();
        Eigen::Vector3d local_theta_des;
        {
            boost::lock_guard<boost::mutex> lock(set_mutex);
            local_theta_des = theta_des;
        }

        auto now = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed_seconds = now - last_time;
        double dt = elapsed_seconds.count();

        if (has_setpoint.load()) {
            jp_type des_handle_theta = compute_pos(local_theta_des);
            Eigen::Vector3d error = des_handle_theta - handle_theta;
            Eigen::Vector3d derivative = (error - prevError) / dt;
            prevError = error;

            jt_type j_torque = (kp_axis * error) + (kd_axis * derivative);
            feedforward_torque = jtmp_matrix * j_torque;
        }
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
        executeControl(feedforward_torque);
        std::this_thread::sleep_for(std::chrono::microseconds(10));
    }

    printf("Entering fault mode!\n");

    for (auto &c : controllers) {
        c->SetBrake();
    }
    return true;
}

bool HapticWrist::executeControl(const mt_type &des_motor_torque) {

    send_frames.clear();

    for (size_t i = 0; i < controllers.size(); i++) {
        cmd.velocity = 0;
        cmd.feedforward_torque = des_motor_torque(i);
        cmd.kp_scale = 0;
        cmd.kd_scale = 0.2;
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

    const auto &v1 = *maybe_servo1;
    const auto &v2 = *maybe_servo2;
    const auto &v3 = *maybe_servo3;
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
