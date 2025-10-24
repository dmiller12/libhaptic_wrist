#include "haptic_wrist/kinematics.h"
#include <cmath>
#include <stdexcept>

namespace haptic_wrist {

Kinematics::Kinematics(std::vector<DHParameter> dh, Eigen::Matrix4d eef_to_tool, Eigen::Matrix4d world_to_base)
    : world_to_base_(world_to_base)
    , dh_params_(dh)
    , eef_to_tool_(eef_to_tool) {
}

std::array<Kin, kWristDofs + 1> Kinematics::eval(haptic_wrist::jp_type pos, const Eigen::Matrix4d& base_to_wrist) {
    if (dh_params_.size() != kWristDofs) {
        throw std::runtime_error("Kinematics::eval expects DH parameters to match wrist DoFs");
    }

    std::array<Kin, kWristDofs + 1> kin;
    Eigen::Matrix4d cumulative_transform = world_to_base_ * base_to_wrist;

    for (size_t i = 0; i < dh_params_.size(); i++) {
        double total_theta = pos(i) + dh_params_[i].theta_pi * M_PI;
        Eigen::Matrix4d link_transform = computeTransform(dh_params_[i], total_theta);
        
        cumulative_transform = cumulative_transform * link_transform;
        kin[i] = Kin{link_transform, cumulative_transform};
    }
    
    Eigen::Matrix4d tool_transform = cumulative_transform * eef_to_tool_;
    kin[kWristDofs] = Kin{eef_to_tool_, tool_transform};

    return kin;
}

std::array<Kin, kWristDofs + 1> Kinematics::eval(const haptic_wrist::jp_type& pos) {
    return eval(pos, Eigen::Matrix4d::Identity());
}

Eigen::Matrix<double, 3, kWristDofs> Kinematics::jacobian_omega(const haptic_wrist::jp_type& pos) {
    if (dh_params_.size() != kWristDofs) {
        throw std::runtime_error("Kinematics::jacobian_omega expects DH parameters to match wrist DoFs");
    }

    Eigen::Matrix<double, 3, kWristDofs> J_omega;

    // The axis of rotation for a revolute joint 'i' is the z-axis of frame 'i-1',
    // expressed in the base frame {0}. J_omega = [z_0, z_1, z_2]
    
    // Get all the forward kinematic transformations.
    // kin[0].to_world_frame contains T_1^0
    // kin[1].to_world_frame contains T_2^0
    // etc.
    // We assume the base frame is the world frame for the Jacobian calculation.
    auto kin = eval(pos, Eigen::Matrix4d::Identity());

    // The axis of rotation for the first joint (joint 1) is the z-axis of the base frame (frame 0).
    J_omega.col(0) << 0, 0, 1;

    for (size_t joint = 1; joint < dh_params_.size(); ++joint) {
        J_omega.col(joint) = kin[joint - 1].to_world_frame.block<3, 3>(0, 0).col(2);
    }

    return J_omega;
}


Eigen::Matrix4d Kinematics::computeTransform(const DHParameter& dh, double theta) {
    // This function correctly implements the standard (Craig) DH transformation.
    double c_theta = cos(theta);
    double s_theta = sin(theta);
    double c_alpha = cos(dh.alpha_pi * M_PI);
    double s_alpha = sin(dh.alpha_pi * M_PI);

    Eigen::Matrix4d T;
    T << c_theta, -s_theta * c_alpha,  s_theta * s_alpha, dh.a * c_theta,
         s_theta,  c_theta * c_alpha, -c_theta * s_alpha, dh.a * s_theta,
         0.0,      s_alpha,            c_alpha,           dh.d,
         0,        0,                  0,                 1;

    return T;
}

}
