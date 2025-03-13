#include "haptic_wrist/haptic_wrist.h"
#include "haptic_wrist_impl.h"

namespace haptic_wrist {

HapticWrist::HapticWrist() : impl(std::make_unique<HapticWristImpl>()) {}

HapticWrist::~HapticWrist() {
    impl->stop();
}

void HapticWrist::setPosition(const jp_type &pos) { impl->setPosition(pos); };

void HapticWrist::setWristToBase(const Eigen::Matrix4d &transform) { impl->setWristToBase(transform); }

jp_type HapticWrist::getPosition() { return impl->getPosition(); }

jv_type HapticWrist::getVelocity() { return impl->getVelocity(); }

jt_type HapticWrist::getTorque() { return impl->getTorque(); }

void HapticWrist::moveTo(const jp_type &desiredPos, double vel, double accel) {
    return impl->moveTo(desiredPos, vel, accel);
}

void HapticWrist::gravityCompensate(bool compensate) { impl->gravityCompensate(compensate); }

void HapticWrist::run() { impl->run(); }

void HapticWrist::stop() { impl->stop(); }

void HapticWrist::hold(bool hold) { impl->hold(hold); }

} // namespace haptic_wrist
