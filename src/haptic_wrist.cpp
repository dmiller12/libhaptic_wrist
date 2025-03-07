#include "haptic_wrist/haptic_wrist.h"
#include "haptic_wrist_impl.h"

namespace haptic_wrist {

HapticWrist::HapticWrist() : impl(std::make_unique<HapticWristImpl>()) {}

HapticWrist::~HapticWrist() {
    impl->stop();
}

void HapticWrist::set_position(const jp_type &pos) { impl->set_position(pos); };

void HapticWrist::set_wrist_to_base(const Eigen::Matrix4d &transform) { impl->set_wrist_to_base(transform); }

jp_type HapticWrist::get_position() { return impl->get_position(); }

jv_type HapticWrist::get_velocity() { return impl->get_velocity(); }

jt_type HapticWrist::get_torque() { return impl->get_torque(); }

void HapticWrist::moveTo(const jp_type &desiredPos, double vel, double accel) {
    return impl->moveTo(desiredPos, vel, accel);
}

void HapticWrist::gravity_compensate(bool compensate) { impl->gravity_compensate(compensate); }

void HapticWrist::run() { impl->run(); }

void HapticWrist::stop() { impl->stop(); }

void HapticWrist::hold(bool hold) { impl->hold(hold); }

} // namespace haptic_wrist
