#include "haptic_wrist/gravity_comp.h"

namespace haptic_wrist {

GravityComp::GravityComp(const Eigen::Matrix<double, kWristDofs, 3>& mus)
    : mus_(mus) {
}

haptic_wrist::jt_type GravityComp::eval(const std::array<Kin, kWristDofs + 1>& kin) {

    auto grav = computeGravity(kin);

    haptic_wrist::jt_type jt = haptic_wrist::jt_type::Zero();
    Eigen::Vector3d prev_torque = Eigen::Vector3d::Zero();
    for (int i = static_cast<int>(kWristDofs) - 1; i >= 0; --i) {
        Eigen::Vector3d mu = mus_.row(i).transpose();
        Eigen::Vector3d t_grav = grav[i].cross(mu);
        t_grav += prev_torque;
        prev_torque = kin[i].to_prev_frame.block<3, 3>(0, 0) * t_grav;
        jt(i) = prev_torque(2);
    }

    return jt;
}

std::array<Eigen::Vector3d, kWristDofs> GravityComp::computeGravity(const std::array<Kin, kWristDofs + 1>& kin) {
    Eigen::Vector3d gravityBase;
    gravityBase << 0, 0, -9.81;
    std::array<Eigen::Vector3d, kWristDofs> grav;
    for (size_t i = 0; i < kWristDofs; i++) {

        Eigen::Matrix3d R = kin[i].to_world_frame.block<3, 3>(0, 0);
        Eigen::Vector3d gravInFrame = R.transpose() * gravityBase;
        grav[i] = gravInFrame;
    }
    return grav;
}
} // namespace haptic_wrist
