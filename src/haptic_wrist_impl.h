#pragma once

#include "moteus.h"
#include <cmath>
#include <math.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "haptic_wrist/gravity_comp.h"
#include "haptic_wrist/kinematics.h"
#include "haptic_wrist/types.h"
#include "joint_position_controller.h"
#include <atomic>
#include <boost/optional.hpp>
#include <boost/thread/locks.hpp>
#include <boost/thread/shared_mutex.hpp>
#include <cstdint>
#include <memory>
#include <thread>
#include <boost/optional.hpp>

namespace haptic_wrist {

// Defines the active control strategy
enum class ControlMode {
    NONE,        // No active control, compliant.
    POSITION,    // Actively controls joint position
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
    boost::optional<handle_type> getHandle();
    double getPassivePosition();
    double getPassiveVelocity();
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
    jp_type position_des_;
    
    // Controllers and Kinematics
    std::unique_ptr<JointPositionController> joint_position_controller_;
    Kinematics kinematics_;
    GravityComp gravity_compensator_;
    
    // Physical state variables
    jp_type handle_theta_;
    jv_type handle_dtheta_;
    jt_type handle_torque_;
    kq_type handle_kin_theta_;
    kv_type handle_kin_dtheta_;
    Eigen::Quaterniond handle_orientation_;
    
    // Configuration and settings
    std::atomic<bool> gravity_compensate_{false};
    Eigen::Matrix4d base_to_wrist_ = Eigen::Matrix4d::Identity();
    double passive_offset_rad_ = 0.0;
    double passive_scale_ = 1.0;

    // Coordinate transformation matrices
    Eigen::Matrix2d jtmp_matrix_; // Joint to motor
    Eigen::Matrix2d mtjp_matrix_; // Motor to joint

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
    static double FindExtraRegister(const mjbots::moteus::Query::Result& result, int16_t register_number);

    static constexpr double radiansPerRotation = 2.0 * M_PI;
};

} // namespace haptic_wrist
