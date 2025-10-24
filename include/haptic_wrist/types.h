#pragma once

#include <Eigen/Dense>

namespace haptic_wrist {
constexpr int kWristDofs = 4;

// Joint-space types
using jp_type = Eigen::Matrix<double, kWristDofs, 1>; // Joint Position
using jv_type = Eigen::Matrix<double, kWristDofs, 1>; // Joint Velocity
using jt_type = Eigen::Matrix<double, kWristDofs, 1>; // Joint Torque

// Motor-space types
using mp_type = Eigen::Matrix<double, kWristDofs, 1>; // Motor Position
using mv_type = Eigen::Matrix<double, kWristDofs, 1>; // Motor Velocity
using mt_type = Eigen::Matrix<double, kWristDofs, 1>; // Motor Torque

// Cartesian-space types (for the end-effector)
using cv_type = Eigen::Vector3d; // Cartesian Velocity (angular)
using ct_type = Eigen::Vector3d; // Cartesian Torque (angular)

} // namespace haptic_wrist
