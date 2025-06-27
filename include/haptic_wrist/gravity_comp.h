#pragma once

#include "haptic_wrist/kinematics.h"
#include "haptic_wrist/types.h"
#include <array>
#include <Eigen/Dense>

namespace haptic_wrist {

class GravityComp {
  public:
    GravityComp();
    GravityComp(const Eigen::Matrix3d& mus);

    /**
     * @brief Computes the gravity compensation torques.
     * @param kin The kinematic chain transformations.
     * @return The 3x1 vector of joint torques to counteract gravity.
     */
    jt_type eval(const std::array<Kin, 3>& kin);

  private:
    /**
     * @brief Helper function to compute the gravity vector for each link.
     * @param kin The kinematic chain transformations.
     * @return An array containing the gravity vector for each of the 3 links.
     */
    static std::array<Eigen::Vector3d, 3> computeGravity(const std::array<Kin, 3>& kin);

    // Matrix of coefficients for the gravity model (link masses and center of mass)
    Eigen::Matrix3d mus_;
    
    // Gravitational acceleration constant (declaration only)
    static const double G;
};

} // namespace haptic_wrist
