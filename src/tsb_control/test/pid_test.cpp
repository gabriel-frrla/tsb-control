#include <cassert>
#include <cmath>
#include <iostream>
#include <stdexcept>

#include "../include/tsb_control/pid.hpp"

int main()
{
    tsb_control::PID controller(2.0, 0.5, 0.1, -10.0, 10.0);

    // First update: P = 2.0, I = 0.05, D = 0.0.
    const double first = controller.compute(1.0, 0.1);
    assert(std::abs(first - 2.05) < 1e-9);

    // Second update: P = 1.6, I = 0.09, D = -0.2.
    const double second = controller.compute(0.8, 0.1);
    assert(std::abs(second - 1.49) < 1e-9);

    // A separate controller must start with its own empty history.
    tsb_control::PID another_controller(2.0, 0.5, 0.1, -10.0, 10.0);
    assert(std::abs(another_controller.compute(1.0, 0.1) - 2.05) < 1e-9);

    // A zero time step must be rejected.
    bool rejected_zero_dt = false;

    try
    {
        controller.compute(1.0, 0.0);
    }
    catch (const std::invalid_argument &)
    {
        rejected_zero_dt = true;
    }

    assert(rejected_zero_dt);

    // Use proportional-only control to isolate output saturation.
    tsb_control::PID limited_controller(2.0, 0.0, 0.0, -1.0, 1.0);

    // Above the upper limit.
    assert(std::abs(limited_controller.compute(2.0, 0.1) - 1.0) < 1e-9);

    // Below the lower limit.
    assert(std::abs(limited_controller.compute(-2.0, 0.1) + 1.0) < 1e-9);

    // Within the limits: output must remain unchanged.
    assert(std::abs(limited_controller.compute(0.2, 0.1) - 0.4) < 1e-9);

    // Sustained saturation must not build up a hidden integral.
    tsb_control::PID antiwindup_controller(1.0, 1.0, 0.0, -1.0, 1.0);

    for (int i = 0; i < 100; ++i)
    {
        assert(
            std::abs(antiwindup_controller.compute(10.0, 0.1) - 1.0) < 1e-9);
    }

    // With zero error, there should be no accumulated integral left over.
    assert(
        std::abs(antiwindup_controller.compute(0.0, 0.1)) < 1e-9);

    // Check the lower limit too.
    for (int i = 0; i < 100; ++i)
    {
        assert(
            std::abs(antiwindup_controller.compute(-10.0, 0.1) + 1.0) < 1e-9);
    }

    assert(
        std::abs(antiwindup_controller.compute(0.0, 0.1)) < 1e-9);

    // Integration must still work and be allowed to move away from a limit.
    tsb_control::PID integral_controller(0.0, 1.0, 0.0, -1.0, 1.0);

    assert(std::abs(integral_controller.compute(0.5, 1.0) - 0.5) < 1e-9);
    assert(std::abs(integral_controller.compute(0.5, 1.0) - 1.0) < 1e-9);
    assert(std::abs(integral_controller.compute(0.5, 1.0) - 1.0) < 1e-9);
    assert(std::abs(integral_controller.compute(-0.2, 1.0) - 0.8) < 1e-9);

    // Isolate the derivative term and test crossing the angle boundary.
    const double pi = std::acos(-1.0);
    const double angle = 179.0 * pi / 180.0;
    const double expected_rate = (2.0 * pi / 180.0) / 0.1;

    tsb_control::PID angular_controller(
    0.0, 0.0, 1.0, -100.0, 100.0, true);

    // First sample: no derivative history.
    assert(std::abs(angular_controller.compute(angle, 0.1)) < 1e-9);

    // +179 to -179 degrees is a +2-degree change.
    assert(
    std::abs(
        angular_controller.compute(-angle, 0.1) - expected_rate) < 1e-9);

    // Reset must discard the previous derivative history.
    angular_controller.reset();
    assert(std::abs(angular_controller.compute(-angle, 0.1)) < 1e-9);

    // Test the opposite crossing: -179 to +179 degrees.
    assert(
    std::abs(
        angular_controller.compute(angle, 0.1) + expected_rate) < 1e-9);

    // Reset must also clear the accumulated integral.
    tsb_control::PID reset_controller(0.0, 1.0, 0.0, -10.0, 10.0);

    assert(std::abs(reset_controller.compute(1.0, 0.1) - 0.1) < 1e-9);

    reset_controller.reset();

    assert(std::abs(reset_controller.compute(0.0, 0.1)) < 1e-9);

    // With zero error, feedforward supplies the requested output.
    tsb_control::PID feedforward_controller(
        0.5, 0.0, 0.0, -0.2, 0.2);

    assert(
        std::abs(
            feedforward_controller.compute(0.0, 0.1, 0.1) - 0.1) < 1e-9);

    // Anti-windup must include feedforward in its saturation decision.
    tsb_control::PID feedforward_antiwindup(
        0.0, 1.0, 0.0, -1.0, 1.0);

    // Proposed output is 0.8 + 0.5 = 1.3.
    // Reject that integral update, leaving the output at 0.8.
    assert(
        std::abs(
            feedforward_antiwindup.compute(0.5, 1.0, 0.8) - 0.8) < 1e-9);

    // Confirm that the rejected integral was not retained.
    assert(
        std::abs(
            feedforward_antiwindup.compute(0.0, 1.0, 0.0)) < 1e-9);

    // With kd = 0, a rapid error change must not falsely block integration.
    tsb_control::PID zero_derivative_gain(
        0.0, 1.0, 0.0, -1.0, 1.0);

    assert(
        std::abs(zero_derivative_gain.compute(0.0, 1.0)) < 1e-9);

    assert(
        std::abs(zero_derivative_gain.compute(0.5, 0.1) - 0.05) < 1e-9);


    // Anti-windup must account for the weighted derivative contribution.
    tsb_control::PID weighted_derivative(
        0.0, 1.0, 2.0, -1.0, 1.0);

    assert(
        std::abs(weighted_derivative.compute(0.0, 1.0)) < 1e-9);

    // D = 2 × (0.06 / 0.1) = 1.2.
    // Positive integration would worsen saturation, so reject it.
    assert(
        std::abs(weighted_derivative.compute(0.06, 0.1) - 1.0) < 1e-9);

    // Error is unchanged, so D = 0.
    // Only this update contributes to the integral: 0.06 × 0.1.
    assert(
        std::abs(weighted_derivative.compute(0.06, 0.1) - 0.006) < 1e-9);

    // Changing ki must preserve the existing integral contribution.
    tsb_control::PID adjustable_integral(
        0.0, 1.0, 0.0, -10.0, 10.0);

    assert(
        std::abs(adjustable_integral.compute(0.2, 1.0) - 0.2) < 1e-9);

    adjustable_integral.set_gains(0.0, 2.0, 0.0);

    assert(
        std::abs(adjustable_integral.compute(0.0, 1.0) - 0.2) < 1e-9);

    // New integration must use the updated gain.
    // Previous contribution: 0.2. New contribution: 2.0 * 0.1 * 1.0.
    assert(
        std::abs(adjustable_integral.compute(0.1, 1.0) - 0.4) < 1e-9);

    // Disabling and re-enabling integration must not restore old history.
    adjustable_integral.set_gains(0.0, 0.0, 0.0);

    assert(
        std::abs(adjustable_integral.compute(0.0, 1.0)) < 1e-9);

    adjustable_integral.set_gains(0.0, 1.0, 0.0);

    assert(
        std::abs(adjustable_integral.compute(0.0, 1.0)) < 1e-9);

    // Gain updates must preserve the previous error used by the derivative.
    tsb_control::PID adjustable_derivative(
        0.0, 0.0, 0.1, -10.0, 10.0);

    assert(
        std::abs(adjustable_derivative.compute(1.0, 0.1)) < 1e-9);

    adjustable_derivative.set_gains(0.0, 0.0, 0.2);

    assert(
        std::abs(adjustable_derivative.compute(1.1, 0.1) - 0.2) < 1e-9);

    std::cout << "All PID checks passed.\n";
    return 0;
}

/*
Compile and Run with:

cd ~/tsb_ws/src/tsb_control

g++ -std=c++17 -Wall -Wextra -Wpedantic \
  -I include \
  test/pid_test.cpp \
  -o /tmp/tsb_pid_test && /tmp/tsb_pid_test

*/