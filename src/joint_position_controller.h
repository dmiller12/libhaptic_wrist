#pragma once

#include "haptic_wrist/types.h"
#include <Eigen/Geometry>
#include <cmath>

namespace haptic_wrist {

/**
 * @class JointPositionController
 * @brief Computes a joint torque to drive joints to a desired position.
 */
class JointPositionController {
  public:
    /**
     * @brief Constructs an JointPositionController.
     * @param kp Proportional gain on orientation error.
     * @param kd Derivative gain on angular velocity (for damping).
     */
    JointPositionController(Eigen::Vector2d kp, Eigen::Vector2d kd, double dt)
        : kp_(kp)
        , kd_(kd)
        , dt_(dt)
        , prevError_(Eigen::Vector2d::Zero()) {
    }

    /**
     * @brief Sets the controller gains.
     */
    void setGains(Eigen::Vector2d kp, Eigen::Vector2d kd) {
        kp_ = kp;
        kd_ = kd;
    }

    /**
     * @brief Computes the control torque
     * @param position_ref The desired joint position (setpoint).
     * @param position_fbk The current joint position (feedback).
     * @return The calculated joint torque.
     */
    Eigen::Vector2d compute_torque(const Eigen::Vector2d& position_ref, const Eigen::Vector2d& position_fbk) {
        Eigen::Vector2d error = position_ref - position_fbk;
        Eigen::Vector2d derivative = (error - prevError_) / dt_;
        prevError_ = error;

        Eigen::Vector2d j_torque = kp_.cwiseProduct(error) + kd_.cwiseProduct(derivative);
        return j_torque;
    }

  private:
    Eigen::Vector2d kp_; // Proportional gain
    Eigen::Vector2d kd_; // Derivative gain
    double dt_;          // time between control cycles
    Eigen::Vector2d prevError_;
};

} // namespace haptic_wrist
