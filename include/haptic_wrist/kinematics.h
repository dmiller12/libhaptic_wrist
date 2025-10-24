#pragma once

#include "haptic_wrist/types.h"
#include <vector>
#include <Eigen/Dense>

namespace haptic_wrist {

struct DHParameter {
    double alpha_pi;
    double a;
    double d;
    double theta_pi = 0.0;
};

struct Kin {
    Eigen::Matrix4d to_prev_frame;
    Eigen::Matrix4d to_world_frame;
};

class Kinematics {
  public:
    Kinematics() = default;
    Kinematics(std::vector<DHParameter> dh, Eigen::Matrix4d eef_to_tool, Eigen::Matrix4d world_to_base);

    /**
     * @brief Evaluates the forward kinematics for the given joint positions.
     * @param pos The joint positions.
     * @param base_to_wrist An optional transformation from the wrist base to the world frame.
     * @return An array of kinematic transformations for each link.
     */
    std::array<Kin, haptic_wrist::kWristDofs + 1> eval(haptic_wrist::jp_type pos,
                                                       const Eigen::Matrix4d& base_to_wrist);
    std::array<Kin, haptic_wrist::kWristDofs + 1> eval(const haptic_wrist::jp_type& pos);

    /**
     * @brief Computes the angular part of the geometric Jacobian.
     * This Jacobian maps joint velocities to the end-effector's angular velocity in the base frame.
     * omega_base = J_omega * q_dot
     * @param pos The current joint positions.
     * @return The 3x3 angular Jacobian matrix.
     */
    Eigen::Matrix<double, 3, haptic_wrist::kWristDofs> jacobian_omega(const haptic_wrist::jp_type& pos);


  private:
    Eigen::Matrix4d world_to_base_;
    std::vector<DHParameter> dh_params_;
    Eigen::Matrix4d eef_to_tool_;
    Eigen::Matrix4d computeTransform(const DHParameter& dh, double theta);
};

}
