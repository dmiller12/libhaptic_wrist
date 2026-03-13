#pragma once

#include "haptic_wrist/kinematics.h"
#include "haptic_wrist/types.h"
#include <Eigen/Dense>
#include <array>

namespace haptic_wrist {

class GravityComp {
  public:
    GravityComp()
        : mus_(Eigen::Matrix<double, 2, 3>::Zero()) {};

    GravityComp(const Eigen::Matrix<double, 2, 3>& mus);

    /**
     * @brief Computes the gravity compensation torques.
     * @param kin The kinematic chain transformations.
     * @return The 3x1 vector of joint torques to counteract gravity.
     */
    jt_type eval(const std::array<Kin, 3>& kin);

    /**
     * @brief Helper function to compute the gravity vector for each link.
     * @param kin The kinematic chain transformations.
     * @return An array containing the gravity vector for each of the 2 links.
     */
    static std::array<Eigen::Vector3d, 2> computeGravity(const std::array<Kin, 3>& kin);

  private:
    // Matrix of coefficients for the gravity model (link masses and center of mass)
    Eigen::Matrix<double, 2, 3> mus_;
};

} // namespace haptic_wrist
