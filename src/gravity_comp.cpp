#include "haptic_wrist/gravity_comp.h"

GravityComp::GravityComp(std::vector<DHParameter> dh, Eigen::Matrix3d mus, Eigen::Matrix4d world_to_base)
    : world_to_base(world_to_base), dh(dh), mus(mus) {}


haptic_wrist::jt_type GravityComp::eval(haptic_wrist::jp_type pos, Eigen::Matrix4d base_to_wrist) {
    
    haptic_wrist::jp_type pos_rearranged;
    pos_rearranged(0) = pos(2);
    pos_rearranged(1) = pos(0);
    pos_rearranged(2) = pos(1);
    Eigen::Matrix4d in_world_frame;
    std::vector<Eigen::Matrix3d> link_prev_frame_R;
    std::vector<Eigen::Vector3d> link_grav;
    in_world_frame = world_to_base * base_to_wrist;

    Eigen::Vector3d gravityBase(0, 0, -9.81);
    for (size_t i = 0; i < dh.size(); i++ ) {
        Eigen::Matrix4d transform = computeTransform(dh[i], pos_rearranged(i));
        link_prev_frame_R.push_back(transform.block<3,3>(0,0));
        in_world_frame = in_world_frame * transform;
        
        Eigen::Matrix3d R = in_world_frame.block<3,3>(0,0);
        Eigen::Vector3d gravInFrame = R.transpose() * gravityBase;
        link_grav.push_back(gravInFrame);
    }

    haptic_wrist::jt_type jt;
    Eigen::Vector3d prev_torque = Eigen::Vector3d::Zero();
    for (size_t i = dh.size() -1; i >= 0; i--) {
        Eigen::Vector3d mu = mus.row(i);    
        Eigen::Vector3d t_grav = link_grav[i].cross(mu) + prev_torque;
        jt(i) = t_grav(2);
        prev_torque = link_prev_frame_R[i] * t_grav;
    }

    haptic_wrist::jt_type jt_rearranged;
    jt_rearranged(0) = jt(1);
    jt_rearranged(1) = jt(2);
    jt_rearranged(2) = jt(0);

    return jt_rearranged; 
}

haptic_wrist::jt_type GravityComp::eval(haptic_wrist::jp_type pos) {
    Eigen::Matrix4d base_to_wrist = Eigen::Matrix4d::Identity();
    return eval(pos, base_to_wrist);
     
}

Eigen::Matrix4d GravityComp::computeTransform(DHParameter dh, double theta) {
    Eigen::Matrix4d T;
    T(0, 0) = cos(theta);
    T(0, 1) = -sin(theta) * cos(dh.alpha_pi * M_PI);
    T(0, 2) = sin(theta) * sin(dh.alpha_pi * M_PI);
    T(0, 3) = dh.a * cos(theta);

    T(1, 0) = sin(theta);
    T(1, 1) = cos(theta) * cos(dh.alpha_pi * M_PI);
    T(1, 2) = -cos(theta) * sin(dh.alpha_pi * M_PI);
    T(1, 3) = dh.a * sin(theta) ;

    T(2, 0) = 0.0;
    T(2, 1) =  sin(dh.alpha_pi * M_PI);
    T(2, 2) =  cos(dh.alpha_pi * M_PI);
    T(2, 3) =  dh.d;

    T(3, 0) =  0;
    T(3, 1) =  0;
    T(3, 2) =  0;
    T(3, 3) =  1;

    return T;
}
