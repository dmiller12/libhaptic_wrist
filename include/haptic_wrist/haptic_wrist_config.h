// HapticWristConfig.h
#pragma once

#include <vector>
#include <Eigen/Dense>
#include "haptic_wrist/kinematics.h"
#include "haptic_wrist/types.h"


struct JointPositionControllerConfig {
    Eigen::Matrix<double, haptic_wrist::kWristDofs, 1> kp;
    Eigen::Matrix<double, haptic_wrist::kWristDofs, 1> kd;
};

struct OrientationControllerConfig {
    double kp = 0.0;
    double kd = 0.0;
};

struct MoteusConfig {
    Eigen::Matrix<double, haptic_wrist::kWristDofs, 1> kd;
    std::vector<std::string> transport_args;
};

struct HapticWristConfig {
    MoteusConfig moteus;
    std::vector<haptic_wrist::DHParameter> dh_parameters;
    Eigen::Matrix4d eef_to_tool;
    Eigen::Matrix<double, haptic_wrist::kWristDofs, haptic_wrist::kWristDofs> j2mp;
    Eigen::Matrix<double, haptic_wrist::kWristDofs, 1> home_position;
    JointPositionControllerConfig joint_position_controller;
    OrientationControllerConfig orientation_controller;
    Eigen::Matrix<double, haptic_wrist::kWristDofs, 3> gravity_mus;
};
