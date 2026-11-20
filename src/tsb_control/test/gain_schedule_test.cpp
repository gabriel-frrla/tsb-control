#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

#include "tsb_control/gain_schedule.hpp"

int main()
{
    const tsb_control::PIDGains low{1.0, 0.2, 0.1};
    const tsb_control::PIDGains high{0.6, 0.4, 0.3};

    const tsb_control::GainSchedule schedule(
        0.05, 0.15, low, high);

    const auto check_gains = [](
        const tsb_control::PIDGains &actual,
        const tsb_control::PIDGains &expected)
    {
        assert(std::abs(actual.kp - expected.kp) < 1e-9);
        assert(std::abs(actual.ki - expected.ki) < 1e-9);
        assert(std::abs(actual.kd - expected.kd) < 1e-9);
    };

    // Below and at the lower threshold.
    check_gains(schedule.gains_for(0.0), low);
    check_gains(schedule.gains_for(0.05), low);

    // Halfway between the thresholds.
    const tsb_control::PIDGains midpoint{0.8, 0.3, 0.2};
    check_gains(schedule.gains_for(0.10), midpoint);

    // At and above the upper threshold.
    check_gains(schedule.gains_for(0.15), high);
    check_gains(schedule.gains_for(0.30), high);

    // Scheduling depends on speed magnitude.
    check_gains(schedule.gains_for(-0.10), midpoint);

    // Equal thresholds would cause division by zero.
    bool rejected_thresholds = false;

    try
    {
        const tsb_control::GainSchedule invalid(
            0.1, 0.1, low, high);
    }
    catch (const std::invalid_argument &)
    {
        rejected_thresholds = true;
    }

    assert(rejected_thresholds);

    // Invalid input must not produce invalid gains.
    bool rejected_nan = false;

    try
    {
        schedule.gains_for(
            std::numeric_limits<double>::quiet_NaN());
    }
    catch (const std::invalid_argument &)
    {
        rejected_nan = true;
    }

    assert(rejected_nan);

    std::cout << "All gain schedule checks passed.\n";
    return 0;
}
