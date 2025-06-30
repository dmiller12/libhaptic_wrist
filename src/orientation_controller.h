#pragma once

#include "haptic_wrist/types.h"
#include <Eigen/Geometry>
#include <cmath>
#include <iostream>

namespace haptic_wrist {

/**
 * @class OrientationController
 * @brief Computes a Cartesian torque to drive the end-effector to a desired orientation.
 *
 * This controller is based on the logic from libbarrett's ToolOrientationController
 * but is adapted to work with a Jacobian that maps to angular velocities in the
 * base frame.
 */
class OrientationController {
public:
    /**
     * @brief Constructs an OrientationController.
     * @param kp Proportional gain on orientation error.
     * @param kd Derivative gain on angular velocity (for damping).
     */
    OrientationController(double kp, double kd) : kp_(kp), kd_(kd) {}

    /**
     * @brief Sets the controller gains.
     */
    void setGains(double kp, double kd) {
        kp_ = kp;
        kd_ = kd;
    }

    /**
     * @brief Computes the control torque in the base frame.
     * @param orientation_ref The desired orientation (setpoint).
     * @param orientation_fbk The current orientation (feedback).
     * @param velocity_fbk_base_frame The current angular velocity in the base frame.
     * @return The calculated 3D Cartesian torque vector in the base frame.
     */
    ct_type compute_torque(
        const Eigen::Quaterniond& orientation_ref,
        const Eigen::Quaterniond& orientation_fbk,
        const cv_type& velocity_fbk_base_frame)
    {
        // Calculate the orientation error as a quaternion.
        // This represents the rotation needed to get from the feedback to the reference.
        Eigen::Quaterniond error_quat = orientation_ref * orientation_fbk.inverse();
        
        // Convert the error to angle-axis representation. The axis is in the base frame.
        Eigen::AngleAxisd error_aa(error_quat);

        double angle = error_aa.angle();
        // Normalize angle to the range [-pi, pi] for shortest path rotation.
        if (angle > M_PI) {
            angle -= 2.0 * M_PI;
        }

        ct_type torque_base_frame;
        // Add a dead-zone near the +/-180 degree discontinuity to prevent erratic behavior.
        if (std::abs(angle) > 3.13) {
            torque_base_frame.setZero();
        } else {
            // Proportional torque is along the error axis, scaled by the error angle and gain.
            // This torque is in the base frame because the error axis is in the base frame.
            ct_type proportional_torque = error_aa.axis() * angle * kp_;
            
            // Add derivative damping. The feedback velocity is already in the base frame.
            torque_base_frame = proportional_torque - (kd_ * velocity_fbk_base_frame);
            // torque_base_frame = proportional_torque;
        }

        return torque_base_frame;
    }

private:
    double kp_; // Proportional gain
    double kd_; // Derivative gain
};

} // namespace haptic_wrist
