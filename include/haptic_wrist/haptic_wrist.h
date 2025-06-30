#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <memory>

#include "haptic_wrist/kinematics.h"
#include "haptic_wrist/types.h"


namespace haptic_wrist {
class HapticWristImpl;

/**
 * @class HapticWrist
 * Control the serial direct drive Z-Y-Z haptic wrist using orientation control.
 * Use run() to start the control loop in a separate thread.
 *
 */
class HapticWrist {
  public:
    HapticWrist();
    ~HapticWrist();

    /**
     * Starts the haptic wrist control loop in a separate thread.
     */
    void run();

    /**
     * Stops the haptic wrist control loop.
     */
    void stop();

    /**
     * @brief Provide a desired orientation for the end-effector.
     * The controller will generate torques to achieve this orientation.
     * @param orientation A quaternion representing the desired orientation in the base frame.
     */
    void setTarget(const Eigen::Quaterniond& orientation);

    /**
     * @brief Provide a desired joint position.
     * The controller will generate torques to achieve this position.
     * @param position Desired joint position in radians.
     */
    void setTarget(const jp_type& position);

    /**
     * @brief Sets the gains for the orientation controller.
     * @param kp Proportional gain on orientation error.
     * @param kd Derivative gain for damping.
     */
    void setOrientationGains(double kp, double kd);

    /**
     * @brief Gets the current orientation of the end-effector.
     * @return A quaternion representing the current orientation in the base frame.
     */
    Eigen::Quaterniond getOrientation();

    /**
     * @brief Enable or disable gravity compensation.
     * @param compensate
     */
    void gravityCompensate(bool compensate = true);

    /**
     * @brief Update the transformation between the wrist and the world frame.
     * @param transform 4x4 homogeneous matrix transforming from wrist base frame to world frame.
     */
    void setWristToBase(const Eigen::Matrix4d& transform);

    /**
     * @brief Commands the wrist to hold its current orientation or release control.
     * @param hold If true, captures the current orientation and holds it.
     * If false, stops applying active control torques (motors will be compliant).
     */
    void hold(bool hold);

    /**
     * @brief Returns the current joint positions.
     * @return Current joint positions [rad]: [Z1, Y2, Z3]
     */
    jp_type getPosition();

    /**
     * @brief Returns the current joint velocities.
     * @return Current joint velocities [rad/s]: [Z1_dot, Y2_dot, Z3_dot]
     */
    jv_type getVelocity();

    /**
     * @brief Returns the last commanded joint torques.
     * @return Current joint torques [N⋅m]: [T_Z1, T_Y2, T_Z3]
     */
    jt_type getTorque();

    /**
     * @brief Returns the kinematics
     * @return Kinematics
     */
    const Kinematics& getKinematics() const;
    
    /**
     * @brief Moves to a desired joint position.
     */
    void moveTo(const jp_type& pos, double vel = 0.5, double accel = 0.5);

    /**
     * @brief Moves to a desired joint position.
     */
    void moveTo(const Eigen::Quaterniond& pos, double vel = 0.5, double accel = 0.5);


  private:
    std::unique_ptr<HapticWristImpl> impl;
};

} // namespace haptic_wrist
