#include "haptic_wrist/trapezoidal_velocity_profile.h"

TrapezoidalVelocityProfile::TrapezoidalVelocityProfile(double vel, double acc, double v_init, double length)
    : vel(vel)
    , acc(acc)
    , v_init(v_init)
    , length(length) {

    double v_diff;

    /* First, is the length too short to decellerate to stop?
     * |\
     * | \
     * |  \__ */
    if (length < 0.5 * v_init * v_init / acc) {
        /* There are no up or plateau phases */
        time_endup = 0.0;
        s_endup = 0.0;
        time_startdown = 0.0;
        s_startdown = 0.0;

        /* The rampdown phase is simple ... */
        time_end = 2 * length / v_init;
        s_end = length;

        /* We need to readjust the acceleration */
        this->acc = v_init / time_end;

        return;
    }

    /* OK, do we not have enough space to plateau?
     * |
     * |/\
     * |  \__ */
    v_diff = vel - v_init;
    if (length < (0.5 * vel * vel + 0.5 * v_diff * v_diff + v_init * v_diff) / acc) {
        double v_top;
        v_top = sqrt(length * acc + 0.5 * v_init * v_init);

        time_endup = (v_top - v_init) / acc;
        s_endup = v_init * time_endup + 0.5 * acc * time_endup * time_endup;

        time_startdown = time_endup;
        s_startdown = s_endup;

        time_end = time_startdown + v_top / acc;
        s_end = s_startdown + 0.5 * v_top * v_top / acc; /* Let's home this is length! */

        return;
    }

    /* OK, we're going to plateau, either up or down
     * | __        |\__
     * |/  \    or |   \
     * |    \__    |    \__ */
    time_endup = fabs(v_diff) / acc;
    s_endup = time_endup * (vel + v_init) / 2;

    /* Compute the ramp down portion */
    s_startdown = length - 0.5 * vel * vel / acc;
    time_startdown = time_endup + (s_startdown - s_endup) / vel;

    s_end = length;
    time_end = time_startdown + vel / acc;

    return;
}

double TrapezoidalVelocityProfile::eval(double t) {
    double s;
    if (t < 0.0) {
        return 0.0;
    }

    /* Are we ramping up? */
    if (t < time_endup) {
        s = v_init * t;
        if (v_init < vel)
            s += 0.5 * acc * t * t;
        else
            s -= 0.5 * acc * t * t;
        return s;
    }
    /* Are we plateau-ed? */
    if (t < time_startdown) {
        s = s_endup + vel * (t - time_endup);
        return s;
    }
    /* Are we ramping down? */
    if (t < time_end) {
        s = s_end - 0.5 * acc * (t - time_end) * (t - time_end);
        return s;
    }

    s = s_end;
    return s;
}

double TrapezoidalVelocityProfile::finalT() {
    return time_end;
}
