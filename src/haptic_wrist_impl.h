#pragma once

#include "moteus.h"
#include <cmath>
#include <math.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "haptic_wrist/gravity_comp.h"
#include "haptic_wrist/kinematics.h"
#include "haptic_wrist/types.h"
#include "orientation_controller.h"
#include "joint_position_controller.h"
#include <atomic>
#include <boost/optional.hpp>
#include <boost/thread/locks.hpp>
#include <boost/thread/shared_mutex.hpp>
#include <memory>
#include <thread>

namespace haptic_wrist {

// Defines the active control strategy
enum class ControlMode {
    NONE,        // No active control, compliant.
    POSITION,    // Actively controls joint position
    ORIENTATION, // Actively controls end-effector orientation.
};

class HapticWristImpl {
  public:
    HapticWristImpl();
    ~HapticWristImpl();
    void run();
    void stop();

    // Control Methods
    void setTarget(const Eigen::Quaterniond& orientation);
    void setTarget(const jp_type& Position);
    void setOrientationGains(double kp, double kd);
    void hold(bool hold);
    void gravityCompensate(bool compensate = true);
    void jointMoveTo(const jp_type& desiredPos, double vel, double accel);
    void moveTo(const Eigen::Quaterniond& desiredOrientation, double vel, double accel);
    void setWristToBase(const Eigen::Matrix4d& transform);


    // Getters
    jp_type getHome() const;
    jp_type getPosition();
    jv_type getVelocity();
    jt_type getTorque();
    mp_type getMotorPositions();
    const Kinematics& getKinematics() const;
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
    jp_type home_;
    const double control_rate_ = 250.0;
    const std::chrono::duration<double> control_period_;
    std::atomic<ControlMode> control_mode_{ControlMode::NONE};
    Eigen::Quaterniond orientation_des_;
    jp_type position_des_;
    
    // Controllers and Kinematics
    std::unique_ptr<OrientationController> orientation_controller_;
    std::unique_ptr<JointPositionController> joint_position_controller_;
    Kinematics kinematics_;
    GravityComp gravity_compensator_;
    
    // Physical state variables
    jp_type handle_theta_;
    jv_type handle_dtheta_;
    jt_type handle_torque_;
    mp_type motor_theta_;
    Eigen::Quaterniond handle_orientation_;
    
    // Configuration and settings
    std::atomic<bool> gravity_compensate_{false};
    Eigen::Matrix4d base_to_wrist_ = Eigen::Matrix4d::Identity();

    // Coordinate transformation matrices
    Eigen::Matrix<double, kWristDofs, kWristDofs> jtmp_matrix_; // Joint to motor
    Eigen::Matrix<double, kWristDofs, kWristDofs> mtjp_matrix_; // Motor to joint

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
