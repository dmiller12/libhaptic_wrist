#include "haptic_wrist/gravity_comp.h"

GravityComp::GravityComp(std::vector<DHParameter> dh, Eigen::Matrix3d mus, Eigen::Matrix4d world_to_base)
    : world_to_base(world_to_base), dh(dh), mus(mus) {}


haptic_wrist::jt_type GravityComp::eval(haptic_wrist::jp_type pos, Eigen::Matrix4d base_to_wrist) {
    return haptic_wrist::jt_type::Zero(); 
}

haptic_wrist::jt_type GravityComp::eval(haptic_wrist::jp_type pos) {
    Eigen::Matrix4d base_to_wrist = Eigen::Matrix4d::Identity();
    return eval(pos, base_to_wrist);
     
}
