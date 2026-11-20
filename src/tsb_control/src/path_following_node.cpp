#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "tsb_interfaces/msg/control_reference.hpp"
#include "tsb_control/los_guidance.hpp"

class PathFollowingNode : public rclcpp::Node
{
public:
    PathFollowingNode()
        : rclcpp::Node("path_following_node"),
          guidance_(declare_startup_parameter<double>(
              "guidance.lookahead", 0.5))
    {
        cruise_speed_ = declare_startup_parameter<double>(
            "guidance.cruise_speed", 0.1);

        acceptance_radius_ = declare_startup_parameter<double>(
            "guidance.acceptance_radius", 0.15);

        approach_gain_ = declare_startup_parameter<double>(
            "guidance.approach_gain", 0.5);

        input_timeout_ = declare_startup_parameter<double>(
            "timing.input_timeout", 0.5);

        const double rate_hz = declare_startup_parameter<double>(
            "timing.rate_hz", 20.0);

        frame_ = declare_startup_parameter<std::string>(
            "frame", "odom");

        const auto xs = declare_startup_parameter<std::vector<double>>(
            "waypoints.x", {0.0, 2.0});

        const auto ys = declare_startup_parameter<std::vector<double>>(
            "waypoints.y", {0.0, 0.0});

        if (!std::isfinite(cruise_speed_) ||
            cruise_speed_ <= 0.0 ||
            cruise_speed_ > 0.2 ||
            !std::isfinite(acceptance_radius_) ||
            acceptance_radius_ <= 0.0 ||
            !std::isfinite(approach_gain_) ||
            approach_gain_ <= 0.0 ||
            !std::isfinite(input_timeout_) ||
            input_timeout_ <= 0.0 ||
            !std::isfinite(rate_hz) ||
            rate_hz < 1.0 ||
            rate_hz > 100.0 ||
            frame_.empty())
        {
            throw std::invalid_argument(
                "Invalid guidance configuration: speed must be "
                "in (0, 0.2] m/s, radius/gain/timeout positive, "
                "rate 1-100 Hz, and frame non-empty.");
        }

        if (xs.size() != ys.size() || xs.size() < 2)
        {
            throw std::invalid_argument(
                "Provide matching x/y arrays with at least two waypoints.");
        }

        for (std::size_t i = 0; i < xs.size(); ++i)
        {
            if (!std::isfinite(xs[i]) || !std::isfinite(ys[i]))
            {
                throw std::invalid_argument(
                    "Waypoint coordinates must be finite.");
            }

            waypoints_.push_back({xs[i], ys[i]});
        }

        for (std::size_t i = 1; i < waypoints_.size(); ++i)
        {
            // Reuse LOS validation to reject invalid segment geometry.
            guidance_.compute(
                waypoints_[i - 1],
                waypoints_[i - 1],
                waypoints_[i]);
        }

        reference_publisher_ =
            this->create_publisher<tsb_interfaces::msg::ControlReference>(
                "control/reference",
                rclcpp::QoS(1));

        state_subscription_ =
            this->create_subscription<nav_msgs::msg::Odometry>(
                "state",
                rclcpp::SensorDataQoS().keep_last(1),
                [this](const nav_msgs::msg::Odometry &message)
                {
                    this->state_callback(message);
                });

        const auto period =
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(1.0 / rate_hz));

        timer_ = this->create_wall_timer(
            period,
            [this]()
            {
                this->guidance_tick();
            });

        RCLCPP_INFO(
            this->get_logger(),
            "Path following initialized with %zu waypoints.",
            waypoints_.size());
    }

private:
    template<typename T>
    T declare_startup_parameter(
        const std::string &name,
        const T &default_value)
    {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.read_only = true;

        return this->declare_parameter<T>(
            name, default_value, descriptor);
    }

    static double wrap_angle(double angle)
    {
        return std::atan2(std::sin(angle), std::cos(angle));
    }

    void state_callback(const nav_msgs::msg::Odometry &message)
    {
        const auto &position = message.pose.pose.position;
        const auto &q = message.pose.pose.orientation;

        const double norm_squared =
            q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;

        const bool valid =
            message.header.frame_id == frame_ &&
            message.child_frame_id == "base_footprint" &&
            message.header.stamp.sec >= 0 &&
            message.header.stamp.nanosec < 1000000000U &&
            std::isfinite(position.x) &&
            std::isfinite(position.y) &&
            std::isfinite(norm_squared) &&
            std::abs(norm_squared - 1.0) <= 1e-3;

        if (!valid)
        {
            has_state_ = false;

            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "State rejected: check frames, position, "
                "quaternion and timestamp.");

            return;
        }

        const double heading = tf2::getYaw(q);

        if (!std::isfinite(heading))
        {
            has_state_ = false;
            return;
        }

        state_ = message;
        heading_ = heading;
        last_state_reception_ = std::chrono::steady_clock::now();
        has_state_ = true;
    }

    void publish_reference(
        double speed,
        double heading,
        const rclcpp::Time &stamp)
    {
        tsb_interfaces::msg::ControlReference reference;

        reference.header.stamp = stamp;
        reference.header.frame_id = frame_;
        reference.surge_velocity = speed;
        reference.heading = wrap_angle(heading);

        reference_publisher_->publish(reference);
    }

    void guidance_tick()
    {
        const auto now = this->now();

        // Restart mission progression after a backward ROS-clock jump.
        if (have_tick_time_ && now < previous_tick_time_)
        {
            has_state_ = false;
            completed_ = false;
            target_index_ = 1;

            RCLCPP_WARN(
                this->get_logger(),
                "Clock moved backward: waiting for new state; mission reset.");
        }

        previous_tick_time_ = now;
        have_tick_time_ = true;

        if (!has_state_ || now.nanoseconds() == 0)
        {
            return;
        }

        const rclcpp::Time state_stamp(
            state_.header.stamp,
            now.get_clock_type());

        const double message_age = (now - state_stamp).seconds();

        const double reception_age =
            std::chrono::duration<double>(
                std::chrono::steady_clock::now() -
                last_state_reception_).count();

        if (message_age < -0.1 ||
            message_age > input_timeout_ ||
            reception_age > input_timeout_)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Guidance inactive: stale state. References withheld.");

            return;
        }

        if (completed_)
        {
            publish_reference(0.0, final_heading_, now);
            return;
        }

        const tsb_control::Point2D position{
            state_.pose.pose.position.x,
            state_.pose.pose.position.y};

        // Advance through any waypoints already inside the acceptance radius.
        while (true)
        {
            const auto &target = waypoints_[target_index_];

            const double distance =
                std::hypot(target.x - position.x, target.y - position.y);

            if (distance > acceptance_radius_)
            {
                break;
            }

            if (target_index_ == waypoints_.size() - 1)
            {
                completed_ = true;
                final_heading_ = heading_;

                publish_reference(0.0, final_heading_, now);

                RCLCPP_INFO(
                    this->get_logger(),
                    "Final waypoint reached. Requesting zero forward speed.");

                return;
            }

            ++target_index_;

            RCLCPP_INFO(
                this->get_logger(),
                "Switching to waypoint index %zu.",
                target_index_);
        }

        const auto &start = waypoints_[target_index_ - 1];
        const auto &end = waypoints_[target_index_];

        const auto result = guidance_.compute(position, start, end);

        double desired_heading = result.heading;

        const double segment_length =
            std::hypot(end.x - start.x, end.y - start.y);

        // If we passed the endpoint without reaching its acceptance region, steer back toward the endpoint instead of following the infinite line.
        if (result.along_track_distance >= segment_length)
        {
            desired_heading =
                std::atan2(end.y - position.y, end.x - position.x);
        }

        const double heading_error =
            wrap_angle(desired_heading - heading_);

        // Reduce forward motion during turns; rotate in place for errors >= 90º
        const double alignment =
            std::max(0.0, std::cos(heading_error));

        // approach_gain has units 1/s, converting distance into a speed limit.
        const double approach_speed =
            approach_gain_ * result.distance_to_end;

        const double speed =
            std::min(cruise_speed_, approach_speed) * alignment;

        publish_reference(speed, desired_heading, now);

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "Target=%zu, cross-track=%.3f m, distance=%.3f m, "
            "speed=%.3f m/s, heading=%.3f rad.",
            target_index_,
            result.cross_track_error,
            result.distance_to_end,
            speed,
            desired_heading);
    }

    tsb_control::LOSGuidance guidance_;
    std::vector<tsb_control::Point2D> waypoints_;

    double cruise_speed_{0.1};
    double acceptance_radius_{0.15};
    double approach_gain_{0.5};
    double input_timeout_{0.5};
    std::string frame_{"odom"};

    nav_msgs::msg::Odometry state_;
    double heading_{0.0};
    bool has_state_{false};

    std::chrono::steady_clock::time_point last_state_reception_;

    std::size_t target_index_{1};
    bool completed_{false};
    double final_heading_{0.0};

    rclcpp::Time previous_tick_time_{0, 0, RCL_ROS_TIME};
    bool have_tick_time_{false};

    rclcpp::Publisher<tsb_interfaces::msg::ControlReference>::SharedPtr
        reference_publisher_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
        state_subscription_;

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    try
    {
        rclcpp::spin(std::make_shared<PathFollowingNode>());
    }
    catch (const std::exception &exception)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("path_following_node"),
            "Path following failed: %s",
            exception.what());

        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
