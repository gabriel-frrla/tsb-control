#pragma once

#include <cmath>
#include <stdexcept>

namespace tsb_control
{

struct Point2D
{
    double x;
    double y;
};

struct LOSResult
{
    double heading;
    double cross_track_error;
    double along_track_distance;
    double distance_to_end;
};

class LOSGuidance
{
public:
    explicit LOSGuidance(double lookahead)
        : lookahead_(lookahead)
    {
        if (!std::isfinite(lookahead_) || lookahead_ <= 0.0)
        {
            throw std::invalid_argument( "LOS lookahead must be finite and positive.");
        }
    }

    LOSResult compute(
        const Point2D &position,
        const Point2D &start,
        const Point2D &end) const
    {
        if (!finite_point(position) ||
            !finite_point(start) ||
            !finite_point(end))
        {
            throw std::invalid_argument( "LOS coordinates must be finite.");
        }

        // Calculate the error and the distance from the start waypoint to the end
        const double dx = end.x - start.x;
        const double dy = end.y - start.y;
        const double length = std::hypot(dx, dy);

        // Reject coincident or effectively coincident waypoints.
        if (!std::isfinite(length) || length <= 1e-6)
        {
            throw std::invalid_argument( "LOS segment length must be finite and greater than 1e-6 m.");
        }

        // Unit vector pointing from the start waypoint to the end.
        const double tangent_x = dx / length;
        const double tangent_y = dy / length;

        const double offset_x = position.x - start.x;
        const double offset_y = position.y - start.y;

        // Positive cross-track error means left of the directed path. Negative error means right of the directed path.
        const double cross_track = -tangent_y * offset_x + tangent_x * offset_y;

        // Project the displacement onto the path direction
        const double along_track = tangent_x * offset_x + tangent_y * offset_y;

        // Calculate the straight distance from the robot to the endpoint
        const double distance_to_end = std::hypot(end.x - position.x, end.y - position.y);

        if (!std::isfinite(cross_track) ||
            !std::isfinite(along_track) ||
            !std::isfinite(distance_to_end))
        {
            throw std::overflow_error( "LOS geometry produced a non-finite result.");
        }

        const double path_heading = std::atan2(dy, dx);

        const double heading = wrap_angle( path_heading - std::atan2(cross_track, lookahead_));

        return LOSResult{
            heading,
            cross_track,
            along_track,
            distance_to_end};
    }

private:
    static bool finite_point(const Point2D &point)
    {
        return std::isfinite(point.x) && std::isfinite(point.y);
    }

    static double wrap_angle(double angle)
    {
        return std::atan2(std::sin(angle), std::cos(angle));
    }

    double lookahead_;
};

}  // namespace tsb_control