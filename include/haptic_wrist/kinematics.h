#pragma once

#include "haptic_wrist/types.h"

struct DHParameter {
    double alpha_pi;
    double a;
    double d;
};

struct Kin {
    Eigen::Matrix4d to_prev_frame;
    Eigen::Matrix4d to_world_frame;
};

class Kinematics {
  public:
    Kinematics() = default;
    Kinematics(std::vector<DHParameter> dh, Eigen::Matrix4d world_to_base);
    std::array<Kin, 3> eval(haptic_wrist::jp_type pos);
    std::array<Kin, 3> eval(haptic_wrist::jp_type pos, Eigen::Matrix4d base_to_wrist);

  private:
    Eigen::Matrix4d world_to_base;
    std::vector<DHParameter> dh_params;
    Eigen::Matrix4d computeTransform(DHParameter dh, double theta);
};
