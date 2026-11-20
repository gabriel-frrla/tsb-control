#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

#include "tsb_control/los_guidance.hpp"

int main()
{
    constexpr double tolerance = 1e-9;
    const double pi = std::acos(-1.0);

    tsb_control::LOSGuidance guidance(1.0);

    const tsb_control::Point2D start{0.0, 0.0};
    const tsb_control::Point2D end{4.0, 0.0};

    // On an eastbound path: command east.
    const auto on_path = guidance.compute({1.0, 0.0}, start, end);

    assert(std::abs(on_path.heading) < tolerance);
    assert(std::abs(on_path.cross_track_error) < tolerance);
    assert(std::abs(on_path.along_track_distance - 1.0) < tolerance);
    assert(std::abs(on_path.distance_to_end - 3.0) < tolerance);

    // Left of the path: steer clockwise toward it.
    const auto left = guidance.compute({1.0, 1.0}, start, end);

    assert(std::abs(left.cross_track_error - 1.0) < tolerance);
    assert(std::abs(left.heading + pi / 4.0) < tolerance);

    // Right of the path: steer counterclockwise toward it.
    const auto right = guidance.compute({1.0, -1.0}, start, end);

    assert(std::abs(right.cross_track_error + 1.0) < tolerance);
    assert(std::abs(right.heading - pi / 4.0) < tolerance);

    // A northbound segment checks that the geometry rotates correctly.
    const auto north =
        guidance.compute({0.0, 1.0}, {0.0, 0.0}, {0.0, 4.0});

    assert(std::abs(north.heading - pi / 2.0) < tolerance);

    // Westbound path: the wrapped heading must remain near pi.
    const auto west =
        guidance.compute({-1.0, 0.0}, {0.0, 0.0}, {-4.0, 0.0});

    assert(std::abs(std::abs(west.heading) - pi) < tolerance);

    // A larger lookahead produces a gentler correction.
    tsb_control::LOSGuidance gentle_guidance(2.0);
    const auto gentle =
        gentle_guidance.compute({1.0, 1.0}, start, end);

    assert(std::abs(gentle.heading) < std::abs(left.heading));

    bool rejected_segment = false;

    try
    {
        guidance.compute({0.0, 0.0}, start, start);
    }
    catch (const std::invalid_argument &)
    {
        rejected_segment = true;
    }

    assert(rejected_segment);

    bool rejected_lookahead = false;

    try
    {
        tsb_control::LOSGuidance invalid_guidance(0.0);
    }
    catch (const std::invalid_argument &)
    {
        rejected_lookahead = true;
    }

    assert(rejected_lookahead);

    bool rejected_nan = false;

    try
    {
        guidance.compute(
            {std::numeric_limits<double>::quiet_NaN(), 0.0},
            start,
            end);
    }
    catch (const std::invalid_argument &)
    {
        rejected_nan = true;
    }

    assert(rejected_nan);

    std::cout << "All LOS guidance checks passed.\n";
    return 0;
}
