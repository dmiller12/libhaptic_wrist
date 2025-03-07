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
    void set_position(const jp_type &pos);
    void gravity_compensate(bool compensate = true);
    void set_wrist_to_base(const Eigen::Matrix4d &transform);
    void hold(bool hold);
    jp_type get_position();
    jv_type get_velocity();
    jt_type get_torque();

    void moveTo(const jp_type &pos, double vel = 0.5, double accel = 0.5);

  private:
    std::unique_ptr<HapticWristImpl> impl;   
};

} // namespace haptic_wrist
