#include "haptic_wrist/haptic_wrist.h"
#include "haptic_wrist_impl.h"
#include <stdexcept>

namespace haptic_wrist {

HapticWrist::HapticWrist()
    : impl(std::make_unique<HapticWristImpl>()) {
}

HapticWrist::~HapticWrist() {
    impl->stop();
}

void HapticWrist::setOrientation(const Eigen::Quaterniond& orientation) {
    impl->setOrientation(orientation);
}

void HapticWrist::setOrientationGains(double kp, double kd) {
    impl->setOrientationGains(kp, kd);
}

Eigen::Quaterniond HapticWrist::getOrientation() {
    return impl->getOrientation();
}

void HapticWrist::setWristToBase(const Eigen::Matrix4d& transform) {
    impl->setWristToBase(transform);
}

jp_type HapticWrist::getPosition() {
    return impl->getPosition();
}

jv_type HapticWrist::getVelocity() {
    return impl->getVelocity();
}

jt_type HapticWrist::getTorque() {
    return impl->getTorque();
}

void HapticWrist::moveTo(const jp_type& desiredPos, double vel, double accel) {
    // This function performs joint-space interpolation and is not compatible
    // with the active orientation controller.
    throw std::runtime_error("moveTo() is not available in orientation control mode.");
}

void HapticWrist::gravityCompensate(bool compensate) {
    impl->gravityCompensate(compensate);
}

void HapticWrist::run() {
    impl->run();
}

void HapticWrist::stop() {
    impl->stop();
}

void HapticWrist::hold(bool hold) {
    impl->hold(hold);
}

} // namespace haptic_wrist
