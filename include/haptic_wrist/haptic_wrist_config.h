// HapticWristConfig.h
#pragma once

#include <vector>
#include <Eigen/Dense>
#include "haptic_wrist/kinematics.h"

struct JointPositionControllerConfig {
    Eigen::Vector3d kp;
    Eigen::Vector3d kd;
};

struct OrientationControllerConfig {
    double kp = 0.0;
    double kd = 0.0;
};

struct MoteusConfig {
    Eigen::Vector2d kd;
    std::string transport_type;
    std::string transport_usb;
    std::string transport_pcie;
};

struct HapticWristConfig {
    MoteusConfig moteus;
    std::vector<haptic_wrist::DHParameter> dh_parameters;
    Eigen::Matrix4d eef_to_tool;
    Eigen::Matrix3d j2mp;
    Eigen::Vector3d home_position;
    JointPositionControllerConfig joint_position_controller;
    OrientationControllerConfig orientation_controller;
    Eigen::Matrix3d gravity_mus;
};
