#pragma once

#include "moteus.h"
#include <cmath>
#include <math.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "haptic_wrist/gravity_comp.h"
#include "haptic_wrist/kinematics.h"
#include "haptic_wrist/orientation_controller.h"
#include "haptic_wrist/types.h"
#include <boost/optional.hpp>
#include <boost/thread/locks.hpp>
#include <boost/thread/shared_mutex.hpp>
#include <thread>
#include <atomic>
#include <memory>

// Gear ratios for serial direct drive
#define MOTOR_TO_JOINT_GEAR_RATIO_1 -1  // Motor 1 to Joint 1 (Z-axis)
#define MOTOR_TO_JOINT_GEAR_RATIO_2 -1 // Motor 2 to Joint 2 (Y-axis)
#define MOTOR_TO_JOINT_GEAR_RATIO_3 -1 // Motor 3 to Joint 3 (Z-axis)

namespace haptic_wrist {

// Defines the active control strategy
enum class ControlMode {
    NONE,           // No active control, compliant.
    ORIENTATION     // Actively controls end-effector orientation.
};

class HapticWristImpl {
  public:
    HapticWristImpl();
    ~HapticWristImpl();
    void run();
    void stop();

    // Control Methods
    void setOrientation(const Eigen::Quaterniond& orientation);
    void setOrientationGains(double kp, double kd);
    void hold(bool hold);
    void gravityCompensate(bool compensate = true);
    void setWristToBase(const Eigen::Matrix4d& transform);

    // Getters
    jp_type getPosition();
    jv_type getVelocity();
    jt_type getTorque();
    Eigen::Quaterniond getOrientation();

  private:
    // Moteus hardware interface
    std::vector<std::shared_ptr<mjbots::moteus::Controller>> controllers_;
    std::shared_ptr<mjbots::moteus::Transport> transport_;
    std::vector<mjbots::moteus::CanFdFrame> send_frames_;
    std::vector<mjbots::moteus::CanFdFrame> receive_frames_;
    mjbots::moteus::PositionMode::Command cmd_;
    int missed_replies_ = 0;

    // Control state
    std::atomic<ControlMode> control_mode_{ControlMode::NONE};
    Eigen::Quaterniond orientation_des_;
    
    // Controllers and Kinematics
    std::unique_ptr<OrientationController> orientation_controller_;
    Kinematics kinematics_;
    GravityComp gravity_compensator_;
    
    // Physical state variables
    jp_type handle_theta_;
    jv_type handle_dtheta_;
    jt_type handle_torque_;
    Eigen::Quaterniond handle_orientation_;
    
    // Configuration and settings
    std::atomic<bool> gravity_compensate_{false};
    Eigen::Matrix4d base_to_wrist_ = Eigen::Matrix4d::Identity();

    // Coordinate transformation matrices
    Eigen::Matrix3d jtmp_matrix_; // Joint to motor
    Eigen::Matrix3d mtjp_matrix_; // Motor to joint

    // Threading and synchronization
    std::atomic<bool> running_{false};
    std::thread control_thread_;
    boost::mutex set_mutex_;
    boost::shared_mutex state_mutex_;

    // Main control loop
    bool entryPoint();
    // Hardware command execution
    bool executeControl(const mt_type& des_motor_torque);
    
    // Helper methods
    jp_type compute_pos(const mp_type& motor_theta);
    jv_type compute_vel(const mv_type& motor_dtheta);
    jt_type compute_torque(const mt_type& motor_torque);
    boost::optional<mjbots::moteus::Query::Result> FindServo(const std::vector<mjbots::moteus::CanFdFrame>& frames, int id);

    static constexpr double radiansPerRotation = 2.0 * M_PI;
};

} // namespace haptic_wrist
