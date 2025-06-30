#pragma once

#include "haptic_wrist/types.h"
class TrapezoidalVelocityProfile {
  public:
    TrapezoidalVelocityProfile(double vel, double acc, double v_init, double length);
    haptic_wrist::jp_type eval();
    double eval(double t);
    double finalT();

  private:
    double vel;
    double acc;
    double v_init;
    double length;

    double time_endup;
    double time_startdown;
    double s_endup;
    double s_startdown;
    double time_end;
    double s_end;
    
};
