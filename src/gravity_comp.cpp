#include "haptic_wrist/gravity_comp.h"

namespace haptic_wrist {

// Definition of the static member variable G
const double GravityComp::G = -9.81;

GravityComp::GravityComp() 
    : mus_(Eigen::Matrix3d::Zero()) {
}

GravityComp::GravityComp(const Eigen::Matrix3d& mus)
    : mus_(mus) {
}

jt_type GravityComp::eval(const std::array<Kin, 4>& kin) {

    std::array<Eigen::Vector3d, 3> grav = computeGravity(kin);

    jt_type jt = jt_type::Zero();
    Eigen::Vector3d prev_torque = Eigen::Vector3d::Zero();
    
    // Iterate backwards from the end-effector to the base
    for (int i = kin.size() - 2; i >= 0; i--) {
        // Get the center of mass vector for the current link
        Eigen::Vector3d mu = mus_.row(i);
        
        // Torque due to gravity on the current link's mass
        Eigen::Vector3d t_grav = grav[i].cross(mu);
        
        // Add torque transmitted from the previous link
        t_grav += prev_torque;

        // Transform the torque into the previous link's frame for the next iteration
        if (i > 0) {
           // Use the transform from link i-1 to i
           prev_torque = kin[i].to_prev_frame.block<3, 3>(0, 0).transpose() * t_grav;
        }
        
        // The joint torque is the z-component of the calculated torque vector
        jt(i) = t_grav(2);
    }

    return jt;
}

std::array<Eigen::Vector3d, 3> GravityComp::computeGravity(const std::array<Kin, 4>& kin) {
    Eigen::Vector3d gravityBase(0, 0, G);
    std::array<Eigen::Vector3d, 3> grav;
    
    for (size_t i = 0; i < kin.size() - 1; i++) {
        // Get the rotation matrix from the world frame to the current link frame
        Eigen::Matrix3d R_world_to_link = kin[i].to_world_frame.block<3, 3>(0, 0);
        
        // Transform the base gravity vector into the current link's frame
        Eigen::Vector3d gravInFrame = R_world_to_link.transpose() * gravityBase;
        grav[i] = gravInFrame;
    }
    return grav;
}

} // namespace haptic_wrist
