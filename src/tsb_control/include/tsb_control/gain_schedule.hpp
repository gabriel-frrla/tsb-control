#pragma once

#include <cmath>
#include <stdexcept>

namespace tsb_control
{
    struct PIDGains
    {
        double kp;
        double ki;
        double kd;
    };

    class GainSchedule
    {
    public:
        GainSchedule(
            double low_speed,
            double high_speed,
            PIDGains low_gains,
            PIDGains high_gains)
            : low_speed_(low_speed),
              high_speed_(high_speed),
              low_gains_(low_gains),
              high_gains_(high_gains)
        {
            // Reject invalid tresholds
            if (!std::isfinite(low_speed_) ||
                !std::isfinite(high_speed_) ||
                low_speed_ < 0.0 ||
                high_speed_ <= low_speed_)
            {
                throw std::invalid_argument(
                    "Speed thresholds must be finite, with "
                    "0 <= low_speed < high_speed.");
            }
            
            // Check for valid PID gains
            validate_gains(low_gains_);
            validate_gains(high_gains_);
        }

        PIDGains gains_for(double surge_reference) const
        {
            if (!std::isfinite(surge_reference))
            {
                throw std::invalid_argument(
                    "Scheduling speed must be finite.");
            }

            const double speed = std::abs(surge_reference);

            if (speed <= low_speed_)
            {
                return low_gains_;
            }

            if (speed >= high_speed_)
            {
                return high_gains_;
            }

            const double alpha =
                (speed - low_speed_) / (high_speed_ - low_speed_);

            return {
                // Interpolate each PID gain independently and return the resulting set
                interpolate(low_gains_.kp, high_gains_.kp, alpha),
                interpolate(low_gains_.ki, high_gains_.ki, alpha),
                interpolate(low_gains_.kd, high_gains_.kd, alpha)
            };
        }

    private:
        static void validate_gains(const PIDGains &gains)
        {
            if (!std::isfinite(gains.kp) ||
                !std::isfinite(gains.ki) ||
                !std::isfinite(gains.kd))
            {
                throw std::invalid_argument(
                    "Scheduled gains must be finite.");
            }
        }

        static double interpolate(double low, double high, double alpha)
        // Calculate a weigheted average of the two endpoint values
        {
            return (1.0 - alpha) * low + alpha * high;
        }

        double low_speed_;
        double high_speed_;
        PIDGains low_gains_;
        PIDGains high_gains_;
    };
}
