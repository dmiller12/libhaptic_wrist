// #pragma once

// #include "moteus.h"
// #include <cmath>
// #include <math.h>

// #include <Eigen/Dense>

// #include "haptic_wrist/gravity_comp.h"
// #include "haptic_wrist/types.h"
// #include <boost/optional.hpp>
// #include <boost/thread/locks.hpp>
// #include <boost/thread/shared_mutex.hpp>
// #include <thread>

// // TODO: check what first var is, unused
// #define MOTOR_TO_HANDLE_SCALE_FACTOR 6.168845556
// #define MOTOR_TO_HANDLE_SCALE_FACTOR_M1 7.46
// #define MOTOR_TO_HANDLE_SCALE_FACTOR_M2 7.46
// #define MOTOR_TO_HANDLE_SCALE_FACTOR_M3 14.87

// namespace haptic_wrist {

// class HapticWristImpl {
//   public:
//     HapticWristImpl();
//     ~HapticWristImpl();
//     void run();
//     void stop();
//     void setPosition(const jp_type& pos);
//     void gravityCompensate(bool compensate = true);
//     void setWristToBase(const Eigen::Matrix4d& transform);
//     void hold(bool hold);
//     jp_type getPosition();
//     jv_type getVelocity();
//     jt_type getTorque();

//     void moveTo(const jp_type& pos, double vel = 0.5, double accel = 0.5);

//   private:
//     std::vector<std::shared_ptr<mjbots::moteus::Controller>> controllers;
//     std::vector<mjbots::moteus::CanFdFrame> send_frames;
//     std::vector<mjbots::moteus::CanFdFrame> receive_frames;
//     jp_type theta_des;
//     jp_type handle_theta;
//     jv_type handle_dtheta;
//     jt_type handle_torque;

//     jp_type j_pos_min;
//     jp_type j_pos_max;

//     mjbots::moteus::PositionMode::Command cmd;
//     Eigen::Matrix3d kp_axis;
//     Eigen::Matrix3d kd_axis;
//     std::shared_ptr<mjbots::moteus::Transport> transport;
//     int missed_replies;
//     std::atomic<bool> gravity;
//     GravityComp gravity_compensator;
//     Kinematics kinematics;

//     static constexpr double radiansPerRotation = 2.0 * M_PI;
//     Eigen::Matrix3d jtmp_matrix;
//     Eigen::Matrix3d mtjp_matrix;
//     boost::optional<mjbots::moteus::Query::Result> FindServo(const std::vector<mjbots::moteus::CanFdFrame>& frames,
//                                                              int id);
//     bool executeControl(const mt_type& motor_torque);
//     bool entryPoint();
//     jp_type compute_pos(const mp_type& motor_theta);
//     jv_type compute_vel(const mv_type& motor_dtheta);
//     jt_type compute_torque(const mt_type& motor_torque);
//     Eigen::Matrix4d baseToWrist = Eigen::Matrix4d::Identity();
//     std::atomic<bool> running{false};
//     std::atomic<bool> has_setpoint{false};
//     std::thread control_thread;
//     boost::mutex set_mutex;
//     boost::shared_mutex state_mutex;
// };

// } // namespace haptic_wrist

#pragma once

#include "moteus.h"
#include <cmath>
#include <math.h>

#include <Eigen/Dense>

#include "haptic_wrist/gravity_comp.h"
#include "haptic_wrist/types.h"
#include <boost/optional.hpp>
#include <boost/thread/locks.hpp>
#include <boost/thread/shared_mutex.hpp>
#include <thread>

// Gear ratios for serial direct drive
#define MOTOR_TO_JOINT_GEAR_RATIO_1 -1  // Motor 1 to Joint 1 (Z-axis)
#define MOTOR_TO_JOINT_GEAR_RATIO_2 -1 // Motor 2 to Joint 2 (Y-axis)
#define MOTOR_TO_JOINT_GEAR_RATIO_3 -1 // Motor 3 to Joint 3 (Z-axis)

namespace haptic_wrist {

class HapticWristImpl {
  public:
    HapticWristImpl();
    ~HapticWristImpl();
    void run();
    void stop();
    void setPosition(const jp_type& pos);
    void gravityCompensate(bool compensate = true);
    void setWristToBase(const Eigen::Matrix4d& transform);
    void hold(bool hold);
    jp_type getPosition();
    jv_type getVelocity();
    jt_type getTorque();

    void moveTo(const jp_type& pos, double vel = 0.5, double accel = 0.5);

  private:
    std::vector<std::shared_ptr<mjbots::moteus::Controller>> controllers;
    std::vector<mjbots::moteus::CanFdFrame> send_frames;
    std::vector<mjbots::moteus::CanFdFrame> receive_frames;
    jp_type theta_des;
    jp_type handle_theta;
    jv_type handle_dtheta;
    jt_type handle_torque;

    jp_type j_pos_min;
    jp_type j_pos_max;

    mjbots::moteus::PositionMode::Command cmd;
    std::shared_ptr<mjbots::moteus::Transport> transport;
    int missed_replies;
    std::atomic<bool> gravity;
    GravityComp gravity_compensator;
    Kinematics kinematics;

    static constexpr double radiansPerRotation = 2.0 * M_PI;
    Eigen::Matrix3d jtmp_matrix;
    Eigen::Matrix3d mtjp_matrix;
    boost::optional<mjbots::moteus::Query::Result> FindServo(const std::vector<mjbots::moteus::CanFdFrame>& frames,
                                                             int id);
    bool executeControl(const mt_type& des_motor_pos, const mt_type& des_motor_torque);
    bool entryPoint();
    jp_type compute_pos(const mp_type& motor_theta);
    jv_type compute_vel(const mv_type& motor_dtheta);
    jt_type compute_torque(const mt_type& motor_torque);
    Eigen::Matrix4d baseToWrist = Eigen::Matrix4d::Identity();
    std::atomic<bool> running{false};
    std::atomic<bool> has_setpoint{false};
    std::thread control_thread;
    boost::mutex set_mutex;
    boost::shared_mutex state_mutex;
};

} // namespace haptic_wrist
