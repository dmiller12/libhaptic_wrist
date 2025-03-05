#include "haptic_wrist/gravity_comp.h"

GravityComp::GravityComp(Eigen::Matrix3d mus) : mus(mus) {}

haptic_wrist::jt_type GravityComp::eval(std::array<Kin, 3> kin) {

    std::array<Eigen::Vector3d, 3> grav = computeGravity(kin);

    haptic_wrist::jt_type jt;
    Eigen::Vector3d prev_torque = Eigen::Vector3d::Zero();
    for (int i = kin.size() - 1; i >= 0; i--) {
        Eigen::Vector3d mu = mus.row(i);
        Eigen::Vector3d t_grav = grav[i].cross(mu);
        t_grav += prev_torque;
        prev_torque = kin[i].to_prev_frame.block<3, 3>(0, 0) * t_grav;
        jt(i) = prev_torque(2);
    }

    haptic_wrist::jt_type jt_rearranged;
    jt_rearranged(0) = jt(1);
    jt_rearranged(1) = jt(2);
    jt_rearranged(2) = jt(0);

    return jt_rearranged;
}

std::array<Eigen::Vector3d, 3> GravityComp::computeGravity(std::array<Kin, 3> kin) {
    Eigen::Vector3d gravityBase(0, 0, -9.81);
    std::array<Eigen::Vector3d, 3> grav;
    for (size_t i = 0; i < kin.size(); i++) {

        Eigen::Matrix3d R = kin[i].to_world_frame.block<3, 3>(0, 0);
        Eigen::Vector3d gravInFrame = R.transpose() * gravityBase;
        grav[i] = gravInFrame;
    }
    return grav;
}
