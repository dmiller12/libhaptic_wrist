#pragma once

#include <Eigen/Dense>
#include <memory>

#include "haptic_wrist/types.h"

// TODO: check what first var is, unused
#define MOTOR_TO_HANDLE_SCALE_FACTOR 6.168845556
#define MOTOR_TO_HANDLE_SCALE_FACTOR_M1 7.46
#define MOTOR_TO_HANDLE_SCALE_FACTOR_M2 7.46
#define MOTOR_TO_HANDLE_SCALE_FACTOR_M3 14.87

namespace haptic_wrist {
class HapticWristImpl;

/**
 * @class HapticWrist
 * Control the haptic wrist. Use run() to start the control loop in a separate thread.
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
     * @param pos desired position.
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
     * @return Current joint positions.
     */
    jp_type getPosition();

    /**
     * Returns the current joint velocities.
     *
     * @return Current joint velocities.
     */
    jv_type getVelocity();

    /**
     * Returns the current joint torques.
     *
     * @return Current joint torques.
     */
    jt_type getTorque();

    /**
     * Moves to the desired position using a trapezoidal velocity profile. Blocks until move is completed.
     *
     * @param pos Desired Position.
     * @param vel Peak velocity.
     * @param accel
     */
    void moveTo(const jp_type& pos, double vel = 0.5, double accel = 0.5);

  private:
    std::unique_ptr<HapticWristImpl> impl;
};

} // namespace haptic_wrist
