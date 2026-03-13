#pragma once

#include <Eigen/Dense>

namespace haptic_wrist {
// Joint-space types
using jp_type = Eigen::Vector2d; // Joint Position
using jv_type = Eigen::Vector2d; // Joint Velocity
using jt_type = Eigen::Vector2d; // Joint Torque

// Motor-space types
using mp_type = Eigen::Vector2d; // Motor Position
using mv_type = Eigen::Vector2d; // Motor Velocity
using mt_type = Eigen::Vector2d; // Motor Torque

// Cartesian-space types (for the end-effector)
using cv_type = Eigen::Vector3d; // Cartesian Velocity (angular)
using ct_type = Eigen::Vector3d; // Cartesian Torque (angular)

} // namespace haptic_wrist
