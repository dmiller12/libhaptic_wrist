#include "haptic_wrist/haptic_wrist.h"
#include "haptic_wrist_impl.h"

namespace haptic_wrist {

HapticWrist::HapticWrist()
    : impl(std::make_unique<HapticWristImpl>()) {
}

HapticWrist::~HapticWrist() {
    impl->stop();
}

void HapticWrist::setTarget(const Eigen::Quaterniond& orientation) {
    impl->setTarget(orientation);
}

void HapticWrist::setOrientationGains(double kp, double kd) {
    impl->setOrientationGains(kp, kd);
}

void HapticWrist::setTarget(const jp_type& position) {
    impl->setTarget(position);
}

Eigen::Quaterniond HapticWrist::getOrientation() {
    return impl->getOrientation();
}

mp_type HapticWrist::getMotorPositions() {
    return impl->getMotorPositions();
}

void HapticWrist::setWristToBase(const Eigen::Matrix4d& transform) {
    impl->setWristToBase(transform);
}

jp_type HapticWrist::getHome() const {
    return impl->getHome();
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

const Kinematics& HapticWrist::getKinematics() const {
    return impl->getKinematics();
}

void HapticWrist::jointMoveTo(const jp_type& desiredPos, double vel, double accel) {
    return impl->jointMoveTo(desiredPos, vel, accel);
}

void HapticWrist::moveTo(const Eigen::Quaterniond& desiredOrientation, double vel, double accel) {
    return impl->moveTo(desiredOrientation, vel, accel);
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
