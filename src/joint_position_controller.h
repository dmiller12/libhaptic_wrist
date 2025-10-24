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
    JointPositionController(const jp_type& kp, const jp_type& kd, double dt)
        : kp_(kp)
        , kd_(kd)
        , dt_(dt)
        , prevError_(jp_type::Zero()) {
    }

    /**
     * @brief Sets the controller gains.
     */
    void setGains(const jp_type& kp, const jp_type& kd) {
        kp_ = kp;
        kd_ = kd;
    }

    /**
     * @brief Computes the control torque
     * @param position_ref The desired joint position (setpoint).
     * @param position_fbk The current joint position (feedback).
     * @return The calculated joint torque.
     */
    jt_type compute_torque(const jp_type& position_ref, const jp_type& position_fbk) {
        jp_type error = position_ref - position_fbk;
        jp_type derivative = (error - prevError_) / dt_;
        prevError_ = error;

        jt_type j_torque = kp_.cwiseProduct(error) + kd_.cwiseProduct(derivative);
        return j_torque;
    }

  private:
    jp_type kp_; // Proportional gain
    jp_type kd_; // Derivative gain
    double dt_;  // time between control cycles
    jp_type prevError_;
};

} // namespace haptic_wrist
