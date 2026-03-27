#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <memory>
#include <boost/optional.hpp>


#include "haptic_wrist/kinematics.h"
#include "haptic_wrist/types.h"


namespace haptic_wrist {
class HapticWristImpl;

/**
 * @class HapticWrist
 * Control the serial direct drive wrist in joint-to-joint mode.
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
     * @brief Orientation targets are disabled in joint-to-joint mode.
     * @param orientation A quaternion representing the desired orientation in the base frame.
     * @throws std::logic_error Always.
     */
    void setTarget(const Eigen::Quaterniond& orientation);

    /**
     * @brief Provide a desired joint position.
     * The controller will generate torques to achieve this position.
     * @param position Desired joint position in radians.
     */
    void setTarget(const jp_type& position);

    /**
     * @brief Orientation control is disabled in joint-to-joint mode.
     * @throws std::logic_error Always.
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
     * @brief Commands the wrist to hold its current joint position or release control.
     * @param hold If true, captures the current active joint positions and holds them.
     * If false, stops applying active control torques (motors will be compliant).
     */
    void hold(bool hold);

    /**
     * @brief Returns the home position.
     * @return Home position [rad] for active joints: [ID1, ID2]
     */
    jp_type getHome() const;

    /**
     * @brief Returns the current joint positions.
     * @return Current joint positions [rad] for active joints: [ID1, ID2]
     */
    jp_type getPosition();

    /**
     * @brief Returns the current joint velocities.
     * @return Current joint velocities [rad/s] for active joints: [ID1_dot, ID2_dot]
     */
    jv_type getVelocity();

    /**
     * @brief Returns the last commanded joint torques.
     * @return Current joint torques [N⋅m] for active joints: [T_ID1, T_ID2]
     */
    jt_type getTorque();

    /**
     * @brief Returns the current handle joystick, bumper and trigger information
     * @return Current handle info: [joystickX, joystickY, bumper, trigger] or null
     */
    boost::optional<handle_type> getHandle();

    /**
     * @brief Returns the passive joint position from AUX2 encoder feedback.
     * @return Passive joint position [rad].
     */
    double getPassivePosition();

    /**
     * @brief Returns the passive joint velocity from AUX2 encoder feedback.
     * @return Passive joint velocity [rad/s].
     */
    double getPassiveVelocity();

    /**
     * @brief Returns the kinematics
     * @return Kinematics
     */
    const Kinematics& getKinematics() const;
    
    /**
     * @brief Moves to a desired joint position.
     */
    void jointMoveTo(const jp_type& pos, double vel = 0.5, double accel = 0.5);

    /**
     * @brief Orientation moves are disabled in joint-to-joint mode.
     * @throws std::logic_error Always.
     */
    void moveTo(const Eigen::Quaterniond& orientation, double vel = 0.5, double accel = 0.5);


  private:
    std::unique_ptr<HapticWristImpl> impl;
};

} // namespace haptic_wrist
