#include <stdio.h>

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <math.h>

#include "haptic_wrist/haptic_wrist.h"

#include <optional>
#include <unistd.h>

const double X_AXIS_MIN_THETA = -M_PI / 2;
const double X_AXIS_MAX_THETA = M_PI / 2;

const double Y_AXIS_MIN_THETA = -0.9;
const double Y_AXIS_MAX_THETA = 0.9;

const double Z_AXIS_MIN_THETA = -1.5;
const double Z_AXIS_MAX_THETA = 1.0;

using namespace mjbots;

namespace haptic_wrist {

HapticWrist::HapticWrist() {

    kp_axis << 0.03, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.1;

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

    moteus::PositionMode::Command cmd;
    cmd.position = 0.0;
    cmd.velocity = 0.0;
    cmd.kp_scale = 1.0;
    cmd.kd_scale = 1.0;
    cmd.feedforward_torque = 0.0;
};

void HapticWrist::set_position(Eigen::Vector3d pos) {
    boost::lock_guard<boost::mutex> lock(set_mutex);
    Eigen::Vector3d wam;
    double x_axis_limited = std::min(std::max(pos(0), X_AXIS_MIN_THETA), X_AXIS_MAX_THETA);
    double y_axis_limited = std::min(std::max(pos(1), Y_AXIS_MIN_THETA), Y_AXIS_MAX_THETA);
    double z_axis_limited = std::min(std::max(pos(2), Z_AXIS_MIN_THETA), Z_AXIS_MAX_THETA);
    wam(0) = -1.0 * x_axis_limited;
    wam(1) = -1.0 * y_axis_limited;
    wam(2) = z_axis_limited;

    theta_des = jtmp() * wam;
};

Eigen::Matrix3d mtjp() {
    Eigen::Matrix3d T;
    T << -0.5 / MOTOR_TO_HANDLE_SCALE_FACTOR_M1, 0.5 / MOTOR_TO_HANDLE_SCALE_FACTOR_M2, 0,
        -0.5 / MOTOR_TO_HANDLE_SCALE_FACTOR_M1, -0.5 / MOTOR_TO_HANDLE_SCALE_FACTOR_M2, 0, 0, 0,
        1 / MOTOR_TO_HANDLE_SCALE_FACTOR_M3;
    return T;
}

Eigen::Matrix3d HapticWrist::jtmp() {
    Eigen::Matrix3d T;
    T << MOTOR_TO_HANDLE_SCALE_FACTOR_M1, MOTOR_TO_HANDLE_SCALE_FACTOR_M1, 0, -MOTOR_TO_HANDLE_SCALE_FACTOR_M2,
        MOTOR_TO_HANDLE_SCALE_FACTOR_M2, 0, 0, 0, MOTOR_TO_HANDLE_SCALE_FACTOR_M3;
    return T;
}

jp_type HapticWrist::compute_pos(const mp_type &motor_theta) { return mtjp() * motor_theta; }

jv_type HapticWrist::compute_vel(const mv_type &motor_dtheta) { return mtjp() * motor_dtheta; }

jt_type HapticWrist::compute_torque(const mt_type &motor_torque) { return KT * mtjp() * motor_torque; }

std::optional<mjbots::moteus::Query::Result>
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

void HapticWrist::run() {
    if (!running) {
        running = true;
        control_thread = boost::thread(&HapticWrist::entryPoint, this);
    }
}

void HapticWrist::stop() {
    running = false;
    if (control_thread.joinable()) {
        control_thread.join();
    }
}

bool HapticWrist::entryPoint() {

    while (running) {
        Eigen::Vector3d local_theta_des;
        {
            boost::lock_guard<boost::mutex> lock(set_mutex);
            local_theta_des = theta_des;
        }

        jp_type des_handle_theta = compute_pos(local_theta_des);
        Eigen::Vector3d p_error = kp_axis * (des_handle_theta - handle_theta);
        p_error(0) = -1.0 * p_error(0);
        p_error(1) = -1.0 * p_error(1);
        mt_type feedforward_torque = jtmp() * p_error;
        executeControl(feedforward_torque);

        // TODO: check how long to sleep for
        boost::this_thread::sleep_for(boost::chrono::milliseconds(20));
    }

    printf("Entering fault mode!\n");

    while (true) {
        // TODO: need sleep in here
        for (auto &c : controllers) {
            c->SetBrake();
        }
    }
}

bool HapticWrist::executeControl(mt_type motor_torque) {

    send_frames.clear();

    for (size_t i = 0; i < controllers.size(); i++) {
        cmd.velocity = 0;
        cmd.feedforward_torque = motor_torque(i);
        cmd.kp_scale = 0;
        cmd.kd_scale = 0.1;
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
    motor_theta(0) = v1.position * 2. * M_PI;
    motor_theta(1) = v2.position * 2. * M_PI;
    motor_theta(2) = v3.position * 2. * M_PI;

    Eigen::Vector3d motor_dtheta;
    motor_dtheta(0) = v1.velocity * 2. * M_PI;
    motor_dtheta(1) = v2.velocity * 2. * M_PI;
    motor_dtheta(2) = v3.velocity * 2. * M_PI;

    Eigen::Vector3d motor_curr;
    motor_curr(0) = v1.torque;
    motor_curr(1) = v2.torque;
    motor_curr(2) = v3.torque;

    {
        boost::unique_lock<boost::shared_mutex> lock(state_mutex);
        handle_theta = compute_pos(motor_theta);
        handle_dtheta = compute_vel(motor_dtheta);
        handle_torque = compute_torque(motor_curr);
    }
    return 0;
}

} // namespace haptic_wrist
