#pragma once

#include <Eigen/Dense>
#include <memory>

#include "haptic_wrist/types.h"


namespace haptic_wrist {
class HapticWristImpl;

/**
 * @class HapticWrist
 * Control the serial direct drive Z-Y-Z haptic wrist. 
 * Use run() to start the control loop in a separate thread.
 *
 */
class HapticWrist {
  public:
    HapticWrist();
    ~HapticWrist();

    /**
     *  Starts the haptic wrist control loop in a separate thread.
     */
    void run();

    /**
     *  Stops the haptic wrist control loop.
     */
    void stop();

    /**
     *  Provide a desired position for real time control. This value is used as the PID reference and should be close
     * to the current position.
     * @see moveTo() for a generated trajectory to a desired position.
     *
     * @param pos desired position in joint space [rad]: [Z1, Y2, Z3]
     */
    void setPosition(const jp_type& pos);

    /**
     * Enable or disable gravity compensation.
     *
     * @param compensate
     */
    void gravityCompensate(bool compensate = true);

    /**
     * Update the transformation between the wrist and the world frame. Useful when used with gravity compensation and
     * base frame wrist movement.
     *
     * @param transform 4x4 homogeneous matrix transforming from wrist base frame to world frame.
     */
    void setWristToBase(const Eigen::Matrix4d& transform);

    /**
     * Holds the current joint positions.
     *
     * @param hold
     */
    void hold(bool hold);

    /**
     * Returns the current joint positions.
     *
     * @return Current joint positions [rad]: [Z1, Y2, Z3]
     */
    jp_type getPosition();

    /**
     * Returns the current joint velocities.
     *
     * @return Current joint velocities [rad/s]: [Z1_dot, Y2_dot, Z3_dot]
     */
    jv_type getVelocity();

    /**
     * Returns the current joint torques.
     *
     * @return Current joint torques [N⋅m]: [T_Z1, T_Y2, T_Z3]
     */
    jt_type getTorque();

    /**
     * Moves to the desired position using a trapezoidal velocity profile. Blocks until move is completed.
     *
     * @param pos Desired Position [rad]: [Z1, Y2, Z3]
     * @param vel Peak velocity [rad/s]
     * @param accel Acceleration [rad/s²]
     */
    void moveTo(const jp_type& pos, double vel = 0.5, double accel = 0.5);

  private:
    std::unique_ptr<HapticWristImpl> impl;
};

} // namespace haptic_wrist