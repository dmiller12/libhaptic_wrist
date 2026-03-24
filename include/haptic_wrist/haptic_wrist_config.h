// HapticWristConfig.h
#pragma once

#include <vector>
#include <Eigen/Dense>
#include "haptic_wrist/kinematics.h"


struct JointPositionControllerConfig {
    Eigen::Vector2d kp;
    Eigen::Vector2d kd;
};

struct OrientationControllerConfig {
    double kp = 0.0;
    double kd = 0.0;
};

struct MoteusConfig {
    Eigen::Vector2d kd;
    std::vector<std::string> transport_args;
};

struct PassiveEncoderConfig {
    double offset_rad = 0.0;
    double scale = 1.0;
};

struct HapticWristConfig {
    MoteusConfig moteus;
    std::vector<haptic_wrist::DHParameter> dh_parameters;
    Eigen::Matrix4d eef_to_tool;
    Eigen::Matrix2d j2mp;
    Eigen::Vector2d home_position;
    JointPositionControllerConfig joint_position_controller;
    OrientationControllerConfig orientation_controller;
    PassiveEncoderConfig passive_encoder;
    Eigen::Matrix<double, 2, 3> gravity_mus;
};
