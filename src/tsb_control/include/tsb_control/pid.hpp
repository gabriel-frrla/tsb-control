#pragma once

#include <cmath>
#include <stdexcept>
#include <algorithm>

namespace tsb_control
{
    class PID
    {
    public:
        explicit PID(double kp, double ki, double kd, double output_min, double output_max, bool angular_error = false)
            : kp_(kp),
              ki_(ki),
              kd_(kd),

              // Saturation
              output_min_(output_min),
              output_max_(output_max),

              prev_error_(0.0),
              integral_(0.0),
              has_previous_error_(false),
              angular_error_(angular_error)
        {
            if (!std::isfinite(kp_) ||
                !std::isfinite(ki_) ||
                !std::isfinite(kd_))
            {
                throw std::invalid_argument("PID gains must be finite.");
            }

            if (!std::isfinite(output_min_) ||
                !std::isfinite(output_max_) ||
                output_min_ >= output_max_)
            {
                throw std::invalid_argument(
                    "PID output limits must be finite, with minimum < maximum.");
            }
        }

        // Compute de PID output
        double compute(double error, double dt, double feedforward = 0.0)
        {

            // Reject invalid inputs before modifying the controller's history.
            if (!std::isfinite(error) || !std::isfinite(dt) || !std::isfinite(feedforward) || dt <= 0.0 )
            {
                throw std::invalid_argument(
                    "PID requires a finite error and a finite, positive dt.");
            }

            const double proportional = kp_ * error;

            // On the first update, there is no previous sample to compare with.
            double derivative = 0.0;

            if (has_previous_error_) {
            double error_change = error - prev_error_;

            // For angular errors, use the shortest signed change.
            if (angular_error_) {
                error_change = std::atan2(
                std::sin(error_change),
                std::cos(error_change));
            }

            derivative = error_change / dt;
            }

            const double derivative_term = kd_ * derivative;

            // Propose an integral update without commiting to it yet. Prevents integral windup by rejecting updates that would push the output beyond the limits.
            double proposed_integral = integral_;

            if (ki_ != 0.0)
            {
                proposed_integral += error * dt;
            }

            const double integral_change = ki_ * (proposed_integral - integral_);

            const double proposed_output = proportional + ki_ * proposed_integral + kd_ * derivative + feedforward;

            // Reject integration that pushed the output beyond the limits.
            const bool pushes_above_max =
                proposed_output > output_max_ && integral_change > 0.0;

            const bool pushes_below_min =
                proposed_output < output_min_ && integral_change < 0.0;

            if (!pushes_above_max && !pushes_below_min)
            {
                integral_ = proposed_integral;
            }

            const double integral_term = ki_ * integral_;

            // Save history for the next update.
            prev_error_ = error;
            has_previous_error_ = true;

            const double output = proportional + integral_term + derivative_term + feedforward;

            return std::clamp(output, output_min_, output_max_); // Returns the output clamped to the specified limits.
        }

        // Update gains while preserving the integral contribution when possible.
        void set_gains(double kp, double ki, double kd)
        {
            if (!std::isfinite(kp) ||
                !std::isfinite(ki) ||
                !std::isfinite(kd))
            {
                throw std::invalid_argument("PID gains must be finite.");
            }

            double updated_integral = integral_;

            if (ki != ki_)
            {
                if (ki == 0.0 || ki_ == 0.0)
                {
                    // Disabling integration clears its history.
                    // Enabling it again starts with no stored contribution.
                    updated_integral = 0.0;
                }
                else
                {
                    // Preserve I: old_ki * old_integral = new_ki * new_integral.
                    const double previous_contribution = ki_ * integral_;
                    updated_integral = previous_contribution / ki;

                    if (!std::isfinite(previous_contribution) ||
                        !std::isfinite(updated_integral))
                    {
                        throw std::invalid_argument(
                            "Gain update would produce an invalid integral state.");
                    }
                }
            }

            // Commit only after all validation succeeds.
            kp_ = kp;
            ki_ = ki;
            kd_ = kd;
            integral_ = updated_integral;
        }

        // Clear controller history while preserving gains and configuration.
        void reset()
        {
        prev_error_ = 0.0;
        integral_ = 0.0;
        has_previous_error_ = false;
        }

    private:
        double kp_;
        double ki_;
        double kd_;

        double output_min_;
        double output_max_;

        double prev_error_;
        double integral_;
        bool has_previous_error_;
        
        bool angular_error_;
    };

} // namespace tsb_control