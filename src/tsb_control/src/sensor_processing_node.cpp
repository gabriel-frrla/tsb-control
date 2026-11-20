#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <initializer_list>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "tf2/LinearMath/Matrix3x3.hpp"
#include "tf2/LinearMath/Quaternion.hpp"

#include "tsb_control/heading_estimator.hpp"

#include <string>

#include  "rcl_interfaces/msg/parameter_descriptor.hpp"
#include <stdexcept>

class SensorProcessingNode : public rclcpp::Node
{
public:
    SensorProcessingNode()
        : rclcpp::Node("sensor_processing_node"),
        heading_time_constant_(declare_numeric_parameter(
            "heading.time_constant", 0.5)),
        input_timeout_(declare_numeric_parameter(
            "timing.input_timeout", 1.0)),
        future_tolerance_(declare_numeric_parameter(
            "timing.future_tolerance", 0.1, true)),
        max_sample_separation_(declare_numeric_parameter(
            "timing.max_sample_separation", 0.1, true)),
        max_dt_(declare_numeric_parameter(
            "timing.max_dt", 0.2)),
        odom_frame_(declare_frame_parameter(
            "frames.odom", "odom")),
        base_frame_(declare_frame_parameter(
            "frames.base", "base_footprint")),
        imu_frame_(declare_frame_parameter(
            "frames.imu", "turtlebot3_burger/imu_link/tb3_imu")),
        heading_estimator_(heading_time_constant_)

    {
        state_publisher_ =
            this->create_publisher<nav_msgs::msg::Odometry>(
                "state",
                rclcpp::QoS(1));
        
        const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(5);

        odometry_subscription_ =
            this->create_subscription<nav_msgs::msg::Odometry>(
                "odom",
                sensor_qos,
                [this](const nav_msgs::msg::Odometry &message)
                {
                    this->odometry_callback(message);
                });

        imu_subscription_ =
            this->create_subscription<sensor_msgs::msg::Imu>(
                "imu",
                sensor_qos,
                [this](const sensor_msgs::msg::Imu &message)
                {
                    this->imu_callback(message);
                });

        RCLCPP_INFO(
            this->get_logger(),
            "Sensor processing node initialized; "
            "waiting for valid odometry and IMU.");
    }

private:
    // Check all components of a position or velocity vector.
    bool finite_vector(double x, double y, double z) const
    {
        return std::isfinite(x) &&
               std::isfinite(y) &&
               std::isfinite(z);
    }

    // Reject non-finite or non-unit quaternions.
    bool valid_quaternion(double x, double y, double z, double w) const
    {
        if (!std::isfinite(x) ||
            !std::isfinite(y) ||
            !std::isfinite(z) ||
            !std::isfinite(w))
        {
            return false;
        }

        const double squared_norm = x * x + y * y + z * z + w * w;

        return std::isfinite(squared_norm) &&
               std::abs(squared_norm - 1.0) <= 1e-3;
    }

    // Check the timestamp against this node's ROS clock.
    bool recent_stamp(int32_t seconds, uint32_t nanoseconds)
    {
        if (seconds < 0 || nanoseconds >= 1000000000U)
        {
            return false;
        }

        const auto now = this->now();

        // Wait until the simulation clock has started.
        if (now.nanoseconds() == 0)
        {
            return false;
        }

        const rclcpp::Time stamp(
            seconds,
            nanoseconds,
            now.get_clock_type());

        const double age = (now - stamp).seconds();

        // Allow a small difference between clock and sensor delivery times.
        return age >= -0.1 && age <= 1.0;
    }

    void odometry_callback(const nav_msgs::msg::Odometry &message)
    {
        const auto &position = message.pose.pose.position;
        const auto &orientation = message.pose.pose.orientation;
        const auto &linear = message.twist.twist.linear;
        const auto &angular = message.twist.twist.angular;

        const bool valid =
            message.header.frame_id == "odom" &&
            message.child_frame_id == "base_footprint" &&
            recent_stamp(
                message.header.stamp.sec,
                message.header.stamp.nanosec) &&
            finite_vector(position.x, position.y, position.z) &&
            finite_vector(linear.x, linear.y, linear.z) &&
            finite_vector(angular.x, angular.y, angular.z) &&
            valid_quaternion(
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w);

        if (!valid)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Odometry rejected: check frames, timestamp, "
                "finite measurements and quaternion.");

            return;
        }

        // Only accepted samples replace the stored measurement.
        latest_odometry_ = message;

        if (!has_odometry_)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "First valid odometry received: "
                "frame='%s', child_frame='%s'.",
                message.header.frame_id.c_str(),
                message.child_frame_id.c_str());
        }

        has_odometry_ = true;
        publish_state_if_ready();
    }

    void imu_callback(const sensor_msgs::msg::Imu &message)
    {
        const auto &orientation = message.orientation;
        const auto &angular = message.angular_velocity;

        // This initial mode requires orientation and angular velocity.
        // A covariance first element of -1 means unavailable.
        const bool orientation_available =
            message.orientation_covariance[0] != -1.0;

        const bool angular_velocity_available =
            message.angular_velocity_covariance[0] != -1.0;

        const bool valid =
            message.header.frame_id ==
                "turtlebot3_burger/imu_link/tb3_imu" &&
            recent_stamp(
                message.header.stamp.sec,
                message.header.stamp.nanosec) &&
            orientation_available &&
            angular_velocity_available &&
            valid_quaternion(
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w) &&
            finite_vector(angular.x, angular.y, angular.z);

        if (!valid)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "IMU rejected: check frame, timestamp, measurement "
                "availability, quaternion and angular velocity.");

            return;
        }

        // Check uncertainty fields for the measurements used by this mode.
        for (std::size_t i = 0; i < 9; ++i)
        {
            if (!std::isfinite(message.orientation_covariance[i]) ||
                !std::isfinite(message.angular_velocity_covariance[i]))
            {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    2000,
                    "IMU rejected: covariance contains non-finite values.");

                return;
            }
        }

        // Covariance diagonal entries represent variances.
        for (const std::size_t i : {0U, 4U, 8U})
        {
            if (message.orientation_covariance[i] < 0.0 ||
                message.angular_velocity_covariance[i] < 0.0)
            {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    2000,
                    "IMU rejected: covariance has a negative variance.");

                return;
            }
        }

        // All-zero covariance means unknown uncertainty, not perfect accuracy.
        latest_imu_ = message;

        if (!has_imu_)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "First valid IMU received: frame='%s'.",
                message.header.frame_id.c_str());
        }

        has_imu_ = true;
    }

    void publish_state_if_ready()
    {
        if (!has_odometry_ || !has_imu_)
        {
            return;
        }

        // Recheck stored measurements before using them.
        if (!recent_stamp(
                latest_odometry_.header.stamp.sec,
                latest_odometry_.header.stamp.nanosec) ||
            !recent_stamp(
                latest_imu_.header.stamp.sec,
                latest_imu_.header.stamp.nanosec))
        {
            reset_heading_estimation();

            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "State withheld: stale sensor data or invalid timestamp.");

            return;
        }

        const auto clock_type = this->get_clock()->get_clock_type();

        const rclcpp::Time odometry_stamp(
            latest_odometry_.header.stamp,
            clock_type);

        const rclcpp::Time imu_stamp(
            latest_imu_.header.stamp,
            clock_type);

        const double sample_separation =
            std::abs((odometry_stamp - imu_stamp).seconds());

        if (sample_separation > 0.1)
        {
            reset_heading_estimation();

            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "State withheld: sensor timestamps differ by %.3f s.",
                sample_separation);

            return;
        }

        // Extract the odometry orientation.
        const auto &odom_orientation = latest_odometry_.pose.pose.orientation;

        tf2::Quaternion odom_quaternion(
            odom_orientation.x,
            odom_orientation.y,
            odom_orientation.z,
            odom_orientation.w);

        odom_quaternion.normalize();

        double odom_roll;
        double odom_pitch;
        double odom_yaw;

        tf2::Matrix3x3(odom_quaternion).getRPY(
            odom_roll,
            odom_pitch,
            odom_yaw);

        // Extract IMU roll and pitch to convert body angular velocity
        // into the rate of change of Euler yaw.
        const auto &imu_orientation = latest_imu_.orientation;

        tf2::Quaternion imu_quaternion(
            imu_orientation.x,
            imu_orientation.y,
            imu_orientation.z,
            imu_orientation.w);

        imu_quaternion.normalize();

        double imu_roll;
        double imu_pitch;
        double imu_yaw;

        tf2::Matrix3x3(imu_quaternion).getRPY(
            imu_roll,
            imu_pitch,
            imu_yaw);

        const double cos_pitch = std::cos(imu_pitch);

        // Euler yaw rate becomes singular near a vertical pitch.
        if (!std::isfinite(cos_pitch) || std::abs(cos_pitch) < 0.1)
        {
            reset_heading_estimation();

            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "State withheld: IMU pitch is too close to the yaw singularity.");

            return;
        }

        // Assumes the IMU axes are aligned with the robot's base axes,
        // as in our current TurtleBot model.
        const double yaw_rate =
            (std::sin(imu_roll) * latest_imu_.angular_velocity.y +
            std::cos(imu_roll) * latest_imu_.angular_velocity.z) /
            cos_pitch;

        if (!std::isfinite(yaw_rate))
        {
            reset_heading_estimation();
            return;
        }

        double fused_heading;

        if (!have_heading_time_)
        {
            fused_heading = heading_estimator_.initialize(odom_yaw);
        }
        else
        {
            const double dt =
                (odometry_stamp - previous_heading_stamp_).seconds();

            // Duplicate measurement: do not integrate or publish it again.
            if (dt == 0.0)
            {
                return;
            }

            // Reinitialize on a subsequent sample after backward timestamps
            // or an interval too large for our rate approximation.
            if (dt < 0.0 || dt > 0.2)
            {
                reset_heading_estimation();

                RCLCPP_WARN_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    2000,
                    "Heading estimator reset: invalid measurement interval %.3f s.",
                    dt);

                return;
            }

            fused_heading =
                heading_estimator_.update(odom_yaw, yaw_rate, dt);
        }

        previous_heading_stamp_ = odometry_stamp;
        have_heading_time_ = true;

        // Preserve odometry position, velocity, frames and timestamp.
        nav_msgs::msg::Odometry state = latest_odometry_;

        // Replace only the heading, retaining odometry roll and pitch.
        tf2::Quaternion fused_orientation;
        fused_orientation.setRPY(
            odom_roll,
            odom_pitch,
            fused_heading);
        fused_orientation.normalize();

        state.pose.pose.orientation.x = fused_orientation.x();
        state.pose.pose.orientation.y = fused_orientation.y();
        state.pose.pose.orientation.z = fused_orientation.z();
        state.pose.pose.orientation.w = fused_orientation.w();

        state_publisher_->publish(state);

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "Heading: odometry=%.3f rad, fused=%.3f rad, IMU yaw rate=%.3f rad/s.",
            odom_yaw,
            fused_heading,
            yaw_rate);
    }

    template<typename T>
    T declare_startup_parameter(
        const std::string &name,
        const T &default_value)
    {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.read_only = true;

        return this->declare_parameter<T>(
            name,
            default_value,
            descriptor);
    }

    double declare_numeric_parameter(
        const std::string &name,
        double default_value,
        bool allow_zero = false)
    {
        const double value =
            declare_startup_parameter<double>(name, default_value);

        if (!std::isfinite(value) ||
            (allow_zero ? value < 0.0 : value <= 0.0))
        {
            throw std::invalid_argument(
                "Parameter '" + name + "' must be finite and " +
                (allow_zero ? "non-negative." : "positive."));
        }

        return value;
    }

    std::string declare_frame_parameter(
        const std::string &name,
        const std::string &default_value)
    {
        const auto value =
            declare_startup_parameter<std::string>(name, default_value);

        if (value.empty() ||
            value.find_first_of(" \t\r\n") != std::string::npos)
        {
            throw std::invalid_argument(
                "Parameter '" + name +
                "' must be a non-empty frame ID without whitespace.");
        }

        return value;
    }

    void reset_heading_estimation()
    {
        heading_estimator_.reset();
        have_heading_time_ = false;
    }
    
    const double heading_time_constant_;
    const double input_timeout_;
    const double future_tolerance_;
    const double max_sample_separation_;
    const double max_dt_;

    const std::string odom_frame_;
    const std::string base_frame_;
    const std::string imu_frame_;

    tsb_control::HeadingEstimator heading_estimator_;

    bool have_heading_time_{false};
    rclcpp::Time previous_heading_stamp_{0, 0, RCL_ROS_TIME};

    // Latest accepted samples. Their age must be rechecked before use.
    nav_msgs::msg::Odometry latest_odometry_;
    sensor_msgs::msg::Imu latest_imu_;

    bool has_odometry_{false};
    bool has_imu_{false};

    // Keep subscriptions alive for the lifetime of the node.
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
        odometry_subscription_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr
        imu_subscription_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr 
        state_publisher_;
    
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    try
    {
        auto node = std::make_shared<SensorProcessingNode>();
        rclcpp::spin(node);
    }
    catch (const std::exception &exception)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("sensor_processing_node"),
            "Sensor processing node failed: %s",
            exception.what());

        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}