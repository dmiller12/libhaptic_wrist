#include "haptic_wrist/kinematics.h"
#include <cmath>
#include <stdexcept>

namespace haptic_wrist {

Kinematics::Kinematics(std::vector<DHParameter> dh, Eigen::Matrix4d eef_to_tool,  Eigen::Matrix4d world_to_base)
    : world_to_base_(world_to_base)
    , dh_params_(dh)
    , eef_to_tool_(eef_to_tool) {
}

std::array<Kin, 4> Kinematics::eval(const haptic_wrist::kq_type& pos, const Eigen::Matrix4d& base_to_wrist) {
    std::array<Kin, 4> kin;
    Eigen::Matrix4d cumulative_transform = world_to_base_ * base_to_wrist;

    if (dh_params_.size() == 2) {
        // Magnum wrist default: implicit first (passive) revolute joint at the wrist base.
        Eigen::Matrix4d passive_transform = computeTransform(DHParameter{}, pos(0));
        cumulative_transform = cumulative_transform * passive_transform;
        kin[0] = Kin{passive_transform, cumulative_transform};

        for (size_t i = 0; i < dh_params_.size(); i++) {
            const double total_theta = pos(i + 1) + dh_params_[i].theta_pi * M_PI;
            const Eigen::Matrix4d link_transform = computeTransform(dh_params_[i], total_theta);

            cumulative_transform = cumulative_transform * link_transform;
            kin[i + 1] = Kin{link_transform, cumulative_transform};
        }
    } else if (dh_params_.size() == 3) {
        // Explicit 3-joint DH chain: [passive, id1, id2].
        for (size_t i = 0; i < dh_params_.size(); i++) {
            const double total_theta = pos(i) + dh_params_[i].theta_pi * M_PI;
            const Eigen::Matrix4d link_transform = computeTransform(dh_params_[i], total_theta);

            cumulative_transform = cumulative_transform * link_transform;
            kin[i] = Kin{link_transform, cumulative_transform};
        }
    } else {
        throw std::runtime_error("Kinematics expects 2 (implicit passive) or 3 DH rows.");
    }

    cumulative_transform = cumulative_transform * eef_to_tool_;
    kin[3] = Kin{eef_to_tool_, cumulative_transform};

    return kin;
}

std::array<Kin, 4> Kinematics::eval(const haptic_wrist::kq_type& pos) {
    return eval(pos, Eigen::Matrix4d::Identity());
}

std::array<Kin, 4> Kinematics::eval(const haptic_wrist::jp_type& active_pos, const Eigen::Matrix4d& base_to_wrist) {
    haptic_wrist::kq_type full_pos;
    full_pos << 0.0, active_pos(0), active_pos(1);
    return eval(full_pos, base_to_wrist);
}

std::array<Kin, 4> Kinematics::eval(const haptic_wrist::jp_type& active_pos) {
    return eval(active_pos, Eigen::Matrix4d::Identity());
}

Eigen::Matrix<double, 3, 3> Kinematics::jacobian_omega(const haptic_wrist::kq_type& pos) {
    Eigen::Matrix<double, 3, 3> J_omega;

    // The axis of rotation for a revolute joint 'i' is the z-axis of frame 'i-1',
    // expressed in the base frame {0}. J_omega = [z_0, z_1, z_2].
    const std::array<Kin, 4> kin = eval(pos, Eigen::Matrix4d::Identity());

    // Joint 0 (passive): z-axis of the base frame.
    J_omega.col(0) << 0, 0, 1;

    // Joint 1 (ID 1): z-axis of frame 0 transformed to base.
    J_omega.col(1) = kin[0].to_world_frame.block<3, 3>(0, 0).col(2);

    // Joint 2 (ID 2): z-axis of frame 1 transformed to base.
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
