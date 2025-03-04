#pragma once

#include "haptic_wrist/types.h"
#include "haptic_wrist/kinematics.h"

class GravityComp {
  public:
    GravityComp() = default;
    GravityComp(Eigen::Matrix3d mus);
    haptic_wrist::jt_type eval(std::array<Kin, 3> kin);
    double gravity = -9.81;
    static std::array<Eigen::Vector3d, 3> computeGravity(std::array<Kin, 3> kin);

  private:
    Eigen::Matrix3d mus;
};
