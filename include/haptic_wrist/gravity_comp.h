#pragma once

#include "haptic_wrist/types.h"

struct DHParameter {
    double alpha_pi;
    double a;
    double d;
};

class GravityComp {
  public:
    GravityComp() = default;
    GravityComp(std::vector<DHParameter> dh, Eigen::Matrix3d mus, Eigen::Matrix4d world_to_base);
    haptic_wrist::jt_type eval(haptic_wrist::jp_type pos);
    haptic_wrist::jt_type eval(haptic_wrist::jp_type pos, Eigen::Matrix4d base_to_wrist);
    double gravity = -9.81;

  private:
    Eigen::Matrix4d world_to_base;
    std::vector<DHParameter> dh;
    Eigen::Matrix3d mus;
};
