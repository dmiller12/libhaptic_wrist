#include "haptic_wrist/kinematics.h"

Kinematics::Kinematics(std::vector<DHParameter> dh, Eigen::Matrix4d world_to_base)
    : world_to_base(world_to_base), dh_params(dh) {}

std::array<Kin, 3> Kinematics::eval(haptic_wrist::jp_type pos, Eigen::Matrix4d base_to_wrist) {

    std::array<Kin, 3> kin;
    Eigen::Matrix4d in_world_frame;
    std::vector<Eigen::Vector3d> link_grav;
    in_world_frame = world_to_base * base_to_wrist;

    for (size_t i = 0; i < dh_params.size(); i++) {
        Eigen::Matrix4d transform = computeTransform(dh_params[i], pos(i));
        in_world_frame = in_world_frame * transform;
        kin[i] = Kin{transform, in_world_frame};
    }
    return kin;
}

std::array<Kin, 3> Kinematics::eval(haptic_wrist::jp_type pos) {
    Eigen::Matrix4d base_to_wrist = Eigen::Matrix4d::Identity();
    return eval(pos, base_to_wrist);
}

Eigen::Matrix4d Kinematics::computeTransform(DHParameter dh, double theta) {
    double c_theta = cos(theta);
    double s_theta = sin(theta);
    double c_alpha = cos(dh.alpha_pi * M_PI);
    double s_alpha = sin(dh.alpha_pi * M_PI);

    Eigen::Matrix4d T;
    T(0, 0) = c_theta;
    T(0, 1) = -s_theta * c_alpha;
    T(0, 2) = s_theta * s_alpha;
    T(0, 3) = dh.a * c_theta;

    T(1, 0) = s_theta;
    T(1, 1) = c_theta * c_alpha;
    T(1, 2) = -c_theta * s_alpha;
    T(1, 3) = dh.a * s_theta;

    T(2, 0) = 0.0;
    T(2, 1) = s_alpha;
    T(2, 2) = c_alpha;
    T(2, 3) = dh.d;

    T(3, 0) = 0;
    T(3, 1) = 0;
    T(3, 2) = 0;
    T(3, 3) = 1;

    return T;
}
