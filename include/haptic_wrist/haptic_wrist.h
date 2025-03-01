#pragma once

#include "moteus.h"
#include <cmath>
#include <math.h>

#include <eigen3/Eigen/Dense>
#include <boost/thread.hpp>

#define KT 0.34641f
#define GR 1.0f

#define MOTOR_TO_HANDLE_SCALE_FACTOR 6.168845556
#define MOTOR_TO_HANDLE_SCALE_FACTOR_M1 7.46
#define MOTOR_TO_HANDLE_SCALE_FACTOR_M2 7.46
#define MOTOR_TO_HANDLE_SCALE_FACTOR_M3 14.87

namespace haptic_wrist {

class HapticWrist {
  public:
    HapticWrist();
	void run();
	void stop();
    void set_position(Eigen::Vector3d pos);
	Eigen::Vector3d get_position();
	Eigen::Vector3d get_velocity();
	Eigen::Vector3d get_torque();

    Eigen::Matrix3d mtjp();
    Eigen::Matrix3d jtmp();

  private:
    std::vector<std::shared_ptr<mjbots::moteus::Controller>> controllers;
    std::vector<mjbots::moteus::CanFdFrame> send_frames;
    std::vector<mjbots::moteus::CanFdFrame> receive_frames;
    Eigen::Vector3d theta_des;
    Eigen::Vector3d handle_theta;
    Eigen::Vector3d handle_dtheta;
    Eigen::Vector3d handle_torque;
    mjbots::moteus::PositionMode::Command cmd;
    Eigen::Matrix3d kp_axis;
    std::shared_ptr<mjbots::moteus::Transport> transport;
    int missed_replies;
    Eigen::VectorXd motor_curr = Eigen::VectorXd::Zero(3);
    Eigen::Vector3d theta_cur;

    std::optional<mjbots::moteus::Query::Result> FindServo(const std::vector<mjbots::moteus::CanFdFrame> &frames,
                                                           int id);
    bool executeControl(Eigen::Vector3d);
    bool entryPoint();
    Eigen::Vector3d compute_pos(const Eigen::VectorXd &_motor_theta);
    Eigen::Vector3d compute_torque(const Eigen::VectorXd &motor_curr);
	boost::atomic<bool> running{false};
	boost::thread control_thread;
	boost::mutex set_mutex;
	boost::shared_mutex state_mutex;
};

} // namespace haptic_wrist
