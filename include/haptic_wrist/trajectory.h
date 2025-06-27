#pragma once

#include <Eigen/Geometry>
#include <chrono>

namespace haptic_wrist {

/**
 * @class Trajectory
 * @brief Generates a smooth orientation trajectory using SLERP.
 *
 * This class handles the interpolation between a start and end orientation
 * over a specified duration.
 */
class Trajectory {
public:
    /**
     * @brief Constructs an orientation trajectory.
     * @param start The starting orientation.
     * @param end The target orientation.
     * @param duration_s The total time the trajectory should take, in seconds.
     */
    Trajectory(const Eigen::Quaterniond& start, const Eigen::Quaterniond& end, double duration_s)
        : start_q_(start.normalized()),
          end_q_(end.normalized()),
          duration_(duration_s),
          elapsed_time_(0.0)
    {
        // Ensure the quaternions are set for the shortest path
        if (start_q_.dot(end_q_) < 0.0) {
            end_q_ = Eigen::Quaterniond(-end_q_.w(), -end_q_.x(), -end_q_.y(), -end_q_.z());
        }
    }

    /**
     * @brief Gets the interpolated orientation setpoint for the current time.
     * @param dt The time elapsed since the last call, in seconds.
     * @return The interpolated quaternion.
     */
    Eigen::Quaterniond get_setpoint(double dt) {
        elapsed_time_ += dt;
        double t = std::min(elapsed_time_ / duration_, 1.0); // Clamp t to [0, 1]
        return start_q_.slerp(t, end_q_);
    }

    /**
     * @brief Checks if the trajectory has completed.
     * @return True if the elapsed time has exceeded the duration.
     */
    bool is_done() const {
        return elapsed_time_ >= duration_;
    }

private:
    Eigen::Quaterniond start_q_;
    Eigen::Quaterniond end_q_;
    double duration_;
    double elapsed_time_;
};

} // namespace haptic_wrist
