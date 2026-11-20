#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

#include "tsb_control/heading_estimator.hpp"

int main()
{
    constexpr double tolerance = 1e-9;
    const double pi = std::acos(-1.0);
    const double degrees_to_radians = pi / 180.0;

    // The first sample anchors the estimate to odometry
    tsb_control::HeadingEstimator estimator(0.5);

    assert(
        std::abs(estimator.update(0.3, 1.0, 0.1) - 0.3) <
        tolerance);

    // Matching odometry and gyro prediction should agree exactly.
    assert(
        std::abs(estimator.update(0.4, 1.0, 0.1) - 0.4) <
        tolerance);

    // Verify prediction and correction when the sensors disagree.
    tsb_control::HeadingEstimator disagreement(0.5);

    assert(
        std::abs(disagreement.update(0.0, 0.0, 0.1)) <
        tolerance);

    // Gyro predicts 0.1 rad; odometry remains at zero.
    // With tau = 0.5 s and dt = 0.1 s, expect about 0.081873 rad.
    assert(
        std::abs(
            disagreement.update(0.0, 1.0, 0.1) -
            0.0818730753077982) < tolerance);

    // Crossing +pi to -pi must represent a small positive rotation.
    tsb_control::HeadingEstimator crossing(0.5);

    const double positive_angle = 179.0 * degrees_to_radians;
    const double negative_angle = -179.0 * degrees_to_radians;
    const double crossing_rate = 2.0 * degrees_to_radians / 0.1;

    crossing.update(positive_angle, 0.0, 0.1);

    assert(
        std::abs(
            crossing.update(negative_angle, crossing_rate, 0.1) -
            negative_angle) < tolerance);

    // Correction across the boundary must also take the short route.
    tsb_control::HeadingEstimator correction_crossing(0.5);
    correction_crossing.update(positive_angle, 0.0, 0.1);

    const double corrected =
        correction_crossing.update(negative_angle, 0.0, 0.1);

    assert(corrected > positive_angle);
    assert(corrected < pi);

    // Reset must discard the previous heading.
    estimator.reset();

    assert(
        std::abs(estimator.update(-0.7, 0.0, 0.1) + 0.7) <
        tolerance);

    // Reject invalid elapsed time.
    bool rejected_zero_dt = false;

    try
    {
        estimator.update(0.0, 0.0, 0.0);
    }
    catch (const std::invalid_argument &)
    {
        rejected_zero_dt = true;
    }

    assert(rejected_zero_dt);

    // Reject non-finite measurements.
    bool rejected_nan = false;

    try
    {
        estimator.update(
            0.0,
            std::numeric_limits<double>::quiet_NaN(),
            0.1);
    }
    catch (const std::invalid_argument &)
    {
        rejected_nan = true;
    }

    assert(rejected_nan);

    // Reject an invalid filter configuration.
    bool rejected_time_constant = false;

    try
    {
        tsb_control::HeadingEstimator invalid_estimator(0.0);
    }
    catch (const std::invalid_argument &)
    {
        rejected_time_constant = true;
    }

    assert(rejected_time_constant);

    std::cout << "All heading estimator checks passed.\n";
    return 0;
}