#include <exception>
#include <memory>
#include <string>

#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

// Timed control loop
#include <chrono>
#include <stdexcept>

#include "tsb_control/pid.hpp"
#include "tsb_interfaces/msg/control_reference.hpp"

// Gain scheduling
#include "tsb_control/gain_schedule.hpp"
#include "geometry_msgs/msg/twist.hpp"

class PidNode : public rclcpp::Node
{
public:
  PidNode()
    : rclcpp::Node("pid_node"),
      surge_pid_(
        declare_startup_parameter("surge.kp", 0.0),
        declare_startup_parameter("surge.ki", 0.0),
        declare_startup_parameter("surge.kd", 0.0),
        declare_startup_parameter("surge.output_min", -1.0),
        declare_startup_parameter("surge.output_max", 1.0)),
      heading_pid_(
        declare_startup_parameter("heading.kp", 0.0),
        declare_startup_parameter("heading.ki", 0.0),
        declare_startup_parameter("heading.kd", 0.0),
        declare_startup_parameter("heading.output_min", -1.0),
        declare_startup_parameter("heading.output_max", 1.0),
        true),
        heading_schedule_(declare_heading_schedule())
      {

    // Subscriber 
    reference_subscription_ =
      this->create_subscription<tsb_interfaces::msg::ControlReference>(
        "control/reference",
        rclcpp::QoS(1),
        [this](const tsb_interfaces::msg::ControlReference & message)
        {
          this->reference_callback(message);
        });

    state_subscription_ =
      this->create_subscription<nav_msgs::msg::Odometry>(
        "state",
        rclcpp::SensorDataQoS().keep_last(1),
        [this](const nav_msgs::msg::Odometry & message)
        {
          this->state_callback(message);
        });

    RCLCPP_INFO(
      this->get_logger(),
      "Surge and heading PID controllers initialized.");

    // Velocity command publisher
    command_publisher_ =
      this->create_publisher<geometry_msgs::msg::Twist>(
        "cmd_vel",
        rclcpp::QoS(1));

    // Control loop timer
    control_rate_hz_ =
      declare_startup_parameter("control.rate_hz", 20.0);

    input_timeout_ =
      declare_startup_parameter("control.input_timeout", 1.0);

    max_dt_ =
      declare_startup_parameter("control.max_dt", 0.2);

    // This implementation supports configured rates from 1 to 1000 Hz.
    if (!std::isfinite(control_rate_hz_) ||
        control_rate_hz_ < 1.0 ||
        control_rate_hz_ > 1000.0 ||
        !std::isfinite(input_timeout_) ||
        input_timeout_ <= 0.0 ||
        !std::isfinite(max_dt_) ||
        max_dt_ <= 1.0 / control_rate_hz_) {
      throw std::invalid_argument(
        "Invalid timing configuration: rate must be 1-1000 Hz, "
        "timeout positive, and max_dt greater than the nominal period.");
    }

    const auto period =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / control_rate_hz_));

    control_timer_ = this->create_wall_timer(
      period,
      [this]()
      {
        this->control_tick();
      });
  }

private:
  // Declare a parameter that can be configured at startup.
  double declare_startup_parameter(
      const std::string &name,
      double default_value)
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;

    return this->declare_parameter<double>(
      name, default_value, descriptor);
  }

  // Read the startup configuration and construct the heading schedule.
  tsb_control::GainSchedule declare_heading_schedule()
  {
      // These parameters were already declared by heading_pid_.
      const tsb_control::PIDGains low_gains{
          this->get_parameter("heading.kp").as_double(),
          this->get_parameter("heading.ki").as_double(),
          this->get_parameter("heading.kd").as_double()
      };

      const double low_speed =
          declare_startup_parameter("heading.schedule.low_speed", 0.0);

      const double high_speed =
          declare_startup_parameter("heading.schedule.high_speed", 0.20);

      // Default to the existing gains to preserve behaviour.
      const tsb_control::PIDGains high_gains{
          declare_startup_parameter(
              "heading.schedule.high.kp", low_gains.kp),
          declare_startup_parameter(
              "heading.schedule.high.ki", low_gains.ki),
          declare_startup_parameter(
              "heading.schedule.high.kd", low_gains.kd)
      };

      return tsb_control::GainSchedule(
          low_speed, high_speed, low_gains, high_gains);
  }

  double surge_error_{0.0};
  double heading_error_{0.0};
  
  // Control loop parameters
  double control_rate_hz_{20.0};
  double input_timeout_{1.0};
  double max_dt_{0.2};

  double surge_command_{0.0};
  double yaw_rate_command_{0.0};

  rclcpp::Time previous_control_time_{0, 0, RCL_ROS_TIME};
  bool has_control_time_{false};

  std::chrono::steady_clock::time_point last_reference_reception_{};
  std::chrono::steady_clock::time_point last_state_reception_{};

  rclcpp::TimerBase::SharedPtr control_timer_;

  // Validate and store a newly received control reference.
  void reference_callback(const tsb_interfaces::msg::ControlReference & message){

    if (!std::isfinite(message.surge_velocity) ||
        !std::isfinite(message.heading)) {
      RCLCPP_WARN(
        this->get_logger(),
        "Reference rejected: speed and heading must be finite.");
      return;
    }

    // Our current interface expects heading expressed in the odom frame.
    if (message.header.frame_id != "odom") {
      RCLCPP_WARN(
        this->get_logger(),
        "Reference rejected: expected frame 'odom', received '%s'.",
        message.header.frame_id.c_str());
      return;
    }

    reference_ = message;
    last_reference_reception_ = std::chrono::steady_clock::now();
    has_reference_ = true;

    RCLCPP_DEBUG(
      this->get_logger(),
      "Reference received: speed = %.3f m/s, heading = %.3f rad.",
      reference_.surge_velocity,
      reference_.heading);
  }

  // Validate and store measured forward speed and heading.
  void state_callback(const nav_msgs::msg::Odometry & message)
  {
    // Pose is expressed in odom; velocity is expressed in base_footprint.
    if (message.header.frame_id != "odom" ||
        message.child_frame_id != "base_footprint") {
      RCLCPP_WARN(
        this->get_logger(),
        "State rejected: expected frames 'odom' and 'base_footprint'.");
      return;
    }

    const double speed = message.twist.twist.linear.x;
    const auto & orientation = message.pose.pose.orientation;

    // Rotation quaternions must have a norm close to one.
    const double norm_squared =
      orientation.x * orientation.x +
      orientation.y * orientation.y +
      orientation.z * orientation.z +
      orientation.w * orientation.w;

    if (!std::isfinite(speed) ||
        !std::isfinite(norm_squared) ||
        std::abs(norm_squared - 1.0) > 1e-3) {
      RCLCPP_WARN(
        this->get_logger(),
        "State rejected: invalid speed or orientation quaternion.");
      return;
    }

    const double heading = tf2::getYaw(orientation);

    if (!std::isfinite(heading)) {
      RCLCPP_WARN(
        this->get_logger(),
        "State rejected: invalid heading.");
      return;
    }

    // Commit the new state only after validation succeeds.
    state_ = message;
    current_surge_velocity_ = speed;
    current_heading_ = heading;
    last_state_reception_ = std::chrono::steady_clock::now();
    has_state_ = true;

    RCLCPP_DEBUG(
      this->get_logger(),
      "State received: speed = %.3f m/s, heading = %.3f rad.",
      current_surge_velocity_,
      current_heading_);
  }

  // Return true only when usable errors have been calculated.
  bool update_errors()
  {
    if (!has_reference_ || !has_state_) {
      return false;
    }

    const double surge_error =
      reference_.surge_velocity - current_surge_velocity_;

    const double heading_difference =
      reference_.heading - current_heading_;

    if (!std::isfinite(surge_error) ||
        !std::isfinite(heading_difference)) {
      return false;
    }

    surge_error_ = surge_error;

    heading_error_ = std::atan2(
      std::sin(heading_difference),
      std::cos(heading_difference));

    return true;
  }

  // Clear controller history and publish a zero velocity command.
  void reset_control(const char * reason)
  {
    surge_pid_.reset();
    heading_pid_.reset();

    surge_command_ = 0.0;
    yaw_rate_command_ = 0.0;

    // NEW: communicate the zero command to downstream subscribers.
    publish_command();

    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      1000,
      "Control inactive: %s. Zero velocity command published.",
      reason);
  }
  void control_tick()
  {
    const rclcpp::Time current_time = this->now();

    // The first tick establishes the time reference.
    if (!has_control_time_) {
      previous_control_time_ = current_time;
      has_control_time_ = true;

      publish_command();  // Both command members initially contain zero.
      return;
    }

    const double dt =
      (current_time - previous_control_time_).seconds();

    previous_control_time_ = current_time;

    // Discard old inputs after a backwards jump or excessive time gap.
    if (dt < 0.0 || dt > max_dt_) {
      has_reference_ = false;
      has_state_ = false;
      reset_control("clock jump or excessive time gap");
      return;
    }

    if (!has_reference_ || !has_state_) {
      reset_control("waiting for reference and state");
      return;
    }

    const auto steady_now = std::chrono::steady_clock::now();

    // Check both the message's timestamp and its local reception age.
    const auto is_fresh =
      [&](const auto & header, const auto & received_at)
      {
        if (header.stamp.sec < 0 ||
            header.stamp.nanosec >= 1000000000u) {
          return false;
        }

        const rclcpp::Time stamp(
          header.stamp, current_time.get_clock_type());

        const double message_age =
          (current_time - stamp).seconds();

        const double reception_age =
          std::chrono::duration<double>(
            steady_now - received_at).count();

        return message_age >= 0.0 &&
              message_age <= input_timeout_ &&
              reception_age <= input_timeout_;
      };

    if (!is_fresh(reference_.header, last_reference_reception_) ||
        !is_fresh(state_.header, last_state_reception_)) {
      reset_control("stale input or invalid timestamp");
      return;
    }

    // Simulation time may pause or remain unchanged between wall ticks.
    // Keep history, but do not integrate without elapsed ROS time.
    if (dt == 0.0) {
      return;
    }

    if (!update_errors()) {
      reset_control("invalid control errors");
      return;
    }

    // Select heading gains using the magnitude of desired forward speed.
    const auto heading_gains =
        heading_schedule_.gains_for(reference_.surge_velocity);

    heading_pid_.set_gains(
        heading_gains.kp,
        heading_gains.ki,
        heading_gains.kd);

    // Surge retains its configured gains and velocity feedforward.
    surge_command_ =
        surge_pid_.compute(surge_error_, dt, reference_.surge_velocity);

    yaw_rate_command_ =
        heading_pid_.compute(heading_error_, dt);

    if (!std::isfinite(surge_command_) ||
        !std::isfinite(yaw_rate_command_)) {
      reset_control("non-finite controller output");
      return;
    }

    publish_command();

    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      1000,
      "Reference speed = %.3f m/s; "
      "heading gains: kp = %.3f, ki = %.3f, kd = %.3f; "
      "commands: v = %.3f m/s, omega = %.3f rad/s.",
      reference_.surge_velocity,
      heading_gains.kp,
      heading_gains.ki,
      heading_gains.kd,
      surge_command_,
      yaw_rate_command_);
  }

  // Publish forward velocity and turning rate.
  void publish_command()
  {
    geometry_msgs::msg::Twist command{};

    command.linear.x = surge_command_;
    command.angular.z = yaw_rate_command_;

    command_publisher_->publish(command);
  }

  // Separate controller objects, each with its own gains and history.
  tsb_control::PID surge_pid_;
  tsb_control::PID heading_pid_;

  // Select heading gains from the desired forward speed
  tsb_control::GainSchedule heading_schedule_;

  // Latest accepted reference and whether one has arrived.
  tsb_interfaces::msg::ControlReference reference_;
  bool has_reference_{false};

  // Keep the subscription alive for the node's lifetime.
  rclcpp::Subscription<
    tsb_interfaces::msg::ControlReference>::SharedPtr reference_subscription_;

  // Latest accepted measured state.
  nav_msgs::msg::Odometry state_;

  double current_surge_velocity_{0.0};
  double current_heading_{0.0};
  bool has_state_{false};

  // Keep the state subscription alive.
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr state_subscription_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_publisher_;

};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  try
  {
    auto node = std::make_shared<PidNode>();
    rclcpp::spin(node);
  }
  catch (const std::exception &exception)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("pid_node"),
      "PID node failed: %s",
      exception.what());

    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
