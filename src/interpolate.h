#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>

/**
 * @brief Primary template for linear interpolation between two points.
 * Works for vector-like types that support +, -, and *double.
 */
template <typename T>
class Interpolate {
public:
    /**
     * @brief Constructs a linear interpolator.
     * @param start The starting point.
     * @param end The ending point.
     */
    Interpolate(const T& start, const T& end)
        : start_(start), end_(end) {
    }

    /**
     * @brief Evaluates the interpolated position.
     * @param s Interpolation parameter from 0.0 (start) to 1.0 (end).
     * @return The interpolated point.
     */
    T eval(double s) const {
        if (s <= 0.0) return start_;
        if (s >= 1.0) return end_;
        // Linear interpolation (lerp)
        return start_ * (1.0 - s) + end_ * s;
    }

    /**
     * @brief Gets the total change (distance) between points.
     * @return The Euclidean distance.
     */
    double getTotalChange() const {
        return (end_ - start_).norm();
    }

private:
    T start_;
    T end_;
};

/**
 * @brief Template specialization for Eigen::Quaterniond.
 * Handles rotational interpolation using slerp.
 */
template <>
class Interpolate<Eigen::Quaterniond> {
public:
    /**
     * @brief Constructs a rotational interpolator.
     * @param start The starting orientation.
     * @param end The ending orientation.
     */
    Interpolate(const Eigen::Quaterniond& start, const Eigen::Quaterniond& end)
        : start_q_(start.normalized()), end_q_(end.normalized()) {
        // Ensure we travel the shortest path
        if (start_q_.dot(end_q_) < 0.0) {
            end_q_.coeffs() *= -1.0;
        }
    }

    /**
     * @brief Evaluates the interpolated orientation.
     * @param s Interpolation parameter from 0.0 (start) to 1.0 (end).
     * @return The interpolated orientation.
     */
    Eigen::Quaterniond eval(double s) const {
        if (s <= 0.0) return start_q_;
        if (s >= 1.0) return end_q_;
        // Spherical linear interpolation (slerp)
        return start_q_.slerp(s, end_q_);
    }

    /**
     * @brief Gets the total change (angle) between orientations.
     * @return The angular distance in radians.
     */
    double getTotalChange() const {
        Eigen::Quaterniond diff = end_q_ * start_q_.inverse();

        Eigen::AngleAxisd diff_aa(diff);
        return diff_aa.angle();
    }

private:
    Eigen::Quaterniond start_q_;
    Eigen::Quaterniond end_q_;
};

