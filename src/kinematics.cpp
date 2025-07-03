#include "haptic_wrist/kinematics.h"
#include <cmath>

namespace haptic_wrist {

Kinematics::Kinematics(std::vector<DHParameter> dh, Eigen::Matrix4d eef_to_tool,  Eigen::Matrix4d world_to_base)
    : world_to_base_(world_to_base)
    , dh_params_(dh)
    , eef_to_tool_(eef_to_tool) {
}

std::array<Kin, 4> Kinematics::eval(haptic_wrist::jp_type pos, const Eigen::Matrix4d& base_to_wrist) {
    std::array<Kin, 4> kin;
    Eigen::Matrix4d cumulative_transform = world_to_base_ * base_to_wrist;

    for (size_t i = 0; i < dh_params_.size(); i++) {
        double total_theta = pos(i);
        Eigen::Matrix4d link_transform = computeTransform(dh_params_[i], total_theta);
        
        cumulative_transform = cumulative_transform * link_transform;
        kin[i] = Kin{link_transform, cumulative_transform};
    }
    
    cumulative_transform = cumulative_transform * eef_to_tool_;
    kin[3] = Kin{eef_to_tool_, cumulative_transform};

    return kin;
}

std::array<Kin, 4> Kinematics::eval(const haptic_wrist::jp_type& pos) {
    return eval(pos, Eigen::Matrix4d::Identity());
}

Eigen::Matrix<double, 3, 3> Kinematics::jacobian_omega(const haptic_wrist::jp_type& pos) {
    Eigen::Matrix<double, 3, 3> J_omega;

    // The axis of rotation for a revolute joint 'i' is the z-axis of frame 'i-1',
    // expressed in the base frame {0}. J_omega = [z_0, z_1, z_2]
    
    // Get all the forward kinematic transformations.
    // kin[0].to_world_frame contains T_1^0
    // kin[1].to_world_frame contains T_2^0
    // etc.
    // We assume the base frame is the world frame for the Jacobian calculation.
    std::array<Kin, 4> kin = eval(pos, Eigen::Matrix4d::Identity());

    // The axis of rotation for the first joint (joint 1) is the z-axis of the base frame (frame 0).
    J_omega.col(0) << 0, 0, 1;

    // The axis of rotation for the second joint (joint 2) is the z-axis of frame 1,
    // expressed in the base frame. This is the third column of the rotation matrix R_1^0.
    J_omega.col(1) = kin[0].to_world_frame.block<3, 3>(0, 0).col(2);

    // The axis of rotation for the third joint (joint 3) is the z-axis of frame 2,
    // expressed in the base frame. This is the third column of the rotation matrix R_2^0.
    J_omega.col(2) = kin[1].to_world_frame.block<3, 3>(0, 0).col(2);

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
