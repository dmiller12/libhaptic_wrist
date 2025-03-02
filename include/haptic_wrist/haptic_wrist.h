#pragma once

#include "moteus.h"
#include <cmath>
#include <math.h>

#include <Eigen/Dense>

#include <mutex>
#include <shared_mutex>
#include <thread>

#define KT 0.34641f
#define GR 1.0f

#define MOTOR_TO_HANDLE_SCALE_FACTOR 6.168845556
#define MOTOR_TO_HANDLE_SCALE_FACTOR_M1 7.46
#define MOTOR_TO_HANDLE_SCALE_FACTOR_M2 7.46
#define MOTOR_TO_HANDLE_SCALE_FACTOR_M3 14.87

namespace haptic_wrist {
using jp_type = Eigen::Vector3d;
using jv_type = Eigen::Vector3d;
using jt_type = Eigen::Vector3d;

using mp_type = Eigen::Vector3d;
using mv_type = Eigen::Vector3d;
using mt_type = Eigen::Vector3d;

class HapticWrist {
  public:
    HapticWrist();
    ~HapticWrist();
    void run();
    void stop();
    void set_position(Eigen::Vector3d pos);
    jp_type get_position();
    jv_type get_velocity();
    jt_type get_torque();

    Eigen::Matrix3d mtjp();
    Eigen::Matrix3d jtmp();

  private:
    std::vector<std::shared_ptr<mjbots::moteus::Controller>> controllers;
    std::vector<mjbots::moteus::CanFdFrame> send_frames;
    std::vector<mjbots::moteus::CanFdFrame> receive_frames;
    jp_type theta_des;
    jp_type handle_theta;
    jv_type handle_dtheta;
    jt_type handle_torque;
    mjbots::moteus::PositionMode::Command cmd;
    Eigen::Matrix3d kp_axis;
    std::shared_ptr<mjbots::moteus::Transport> transport;
    int missed_replies;

    std::optional<mjbots::moteus::Query::Result> FindServo(const std::vector<mjbots::moteus::CanFdFrame> &frames,
                                                           int id);
    bool executeControl(mt_type motor_torque);
    bool entryPoint();
    jp_type compute_pos(const mp_type &motor_theta);
    jv_type compute_vel(const mv_type &motor_dtheta);
    jt_type compute_torque(const mt_type &motor_torque);
    std::atomic<bool> running{false};
    std::thread control_thread;
    std::mutex set_mutex;
    std::shared_mutex state_mutex;
};

} // namespace haptic_wrist
