#pragma once

#include "haptic_wrist/interpolate.h"
#include "haptic_wrist/trapezoidal_velocity_profile.h"

namespace haptic_wrist {

/**
 * @brief Timing and interpolation helper for point-to-point motions.
 */
template <typename T>
class Trajectory {
public:
    /**
     * @brief Constructs a point-to-point trajectory for any supported type.
     * @param start The starting point (e.g., jp_type or Quaterniond).
     * @param end The ending point.
     * @param max_vel The maximum velocity (m/s or rad/s).
     * @param max_acc The maximum acceleration (m/s^2 or rad/s^2).
     */
    Trajectory(const T& start, const T& end, double max_vel, double max_acc)
        : interpolator_(start, end)
        , profile_(max_vel, max_acc, 0.0, interpolator_.getTotalChange()) {}

    /**
     * @brief Gets the interpolated setpoint at a specific absolute time.
     * @param t The time in seconds since the trajectory began.
     * @return The calculated setpoint.
     */
    T get_setpoint_at(double t) {
        // 1. Get the un-normalized distance 's' from the timing profile.
        double s_distance = profile_.eval(t);

        // 2. Normalize 's' to a [0, 1] parameter for the interpolator.
        double total_change = interpolator_.getTotalChange();
        if (total_change < 1e-9) { // Avoid division by zero
            return interpolator_.eval(1.0);
        }
        double s_parameter = s_distance / total_change;

        // 3. Evaluate the spatial interpolator with the normalized parameter.
        return interpolator_.eval(s_parameter);
    }

    /**
     * @brief Gets the total calculated duration of the trajectory.
     * @return The duration in seconds.
     */
    double get_duration() {
        return profile_.finalT();
    }

private:
    Interpolate<T> interpolator_;
    TrapezoidalVelocityProfile profile_;
};

} // namespace haptic_wrist
