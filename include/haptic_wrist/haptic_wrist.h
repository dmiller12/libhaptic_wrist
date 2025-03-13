#pragma once

#include <Eigen/Dense>
#include <memory>

#include "haptic_wrist/types.h"

// TODO: check what first var is, unused
#define MOTOR_TO_HANDLE_SCALE_FACTOR 6.168845556
#define MOTOR_TO_HANDLE_SCALE_FACTOR_M1 7.46
#define MOTOR_TO_HANDLE_SCALE_FACTOR_M2 7.46
#define MOTOR_TO_HANDLE_SCALE_FACTOR_M3 14.87

namespace haptic_wrist {
class HapticWristImpl;

class HapticWrist {
  public:
    HapticWrist();
    ~HapticWrist();
    void run();
    void stop();
    void setPosition(const jp_type &pos);
    void gravityCompensate(bool compensate = true);
    void setWristToBase(const Eigen::Matrix4d &transform);
    void hold(bool hold);
    jp_type getPosition();
    jv_type getVelocity();
    jt_type getTorque();

    void moveTo(const jp_type &pos, double vel = 0.5, double accel = 0.5);

  private:
    std::unique_ptr<HapticWristImpl> impl;   
};

} // namespace haptic_wrist
