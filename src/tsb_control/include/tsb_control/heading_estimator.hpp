#pragma once

#include <cmath>
#include <stdexcept>

namespace tsb_control
{

class HeadingEstimator
{
public:
    // Configurem how quickly the estimate is corrected toward odometry: larger \tau -> slower correction
    explicit HeadingEstimator(double correction_time_constant)
        : correction_time_constant_(correction_time_constant)
    {
        if (!std::isfinite(correction_time_constant_) ||
            correction_time_constant_ <= 0.0)
        {
            throw std::invalid_argument("Heading correction time constant must be finite and positive.");
        }
    }

    // odometry_heading: heading in the odom frame, in radians.
    // heading_rate: rate of change of heading, in radians per second.
    // dt: elapsed measurement time, in seconds.
    //
    // The first update anchors the estimate to odometry.
    double update(double odometry_heading, double heading_rate, double dt)
    {
        if (!std::isfinite(odometry_heading) ||
            !std::isfinite(heading_rate) ||
            !std::isfinite(dt) ||
            dt <= 0.0)
        {
            throw std::invalid_argument(
                "Heading estimator requires finite measurements "
                "and a finite, positive dt.");
        }

        const double measured_heading = wrap_angle(odometry_heading);

        // Start with odomotry when no precious heading estimate exists
        if (!initialized_)
        {
            heading_ = measured_heading;
            initialized_ = true;
            return heading_;
        }

        // Predict heading using the supplied angular rate: predicted heading = previous estimate + heading rate × elapsed time
        const double increment = heading_rate * dt;
        const double unwrapped_prediction = heading_ + increment;

        if (!std::isfinite(increment) ||
            !std::isfinite(unwrapped_prediction))
        {
            throw std::overflow_error(
                "Heading prediction exceeded the finite numeric range.");
        }

        const double predicted_heading = wrap_angle(unwrapped_prediction);

        // Use the shortest angular difference for the correction.
        const double correction_error =
            wrap_angle(measured_heading - predicted_heading);

        // A larger time constant produces slower correction to odometry.
        const double alpha =
            std::exp(-dt / correction_time_constant_);

        const double corrected_heading =
            predicted_heading + (1.0 - alpha) * correction_error;

        heading_ = wrap_angle(corrected_heading);
        return heading_;
    }

    // Re-anchor to odometry on the next update.
    void reset()
    {
        heading_ = 0.0;
        initialized_ = false;
    }

    // Anchor the estimate without inventing an initial time interval.
    double initialize(double heading)
    {
        if (!std::isfinite(heading))
        {
            throw std::invalid_argument(
                "Initial heading must be finite.");
        }

        
    heading_ = wrap_angle(heading);
    initialized_ = true;
    return heading_;
}

private:
    static double wrap_angle(double angle)
    {
        return std::atan2(std::sin(angle), std::cos(angle));
    }

    double correction_time_constant_;
    double heading_{0.0};
    bool initialized_{false};
};

} // namespace tsb_control