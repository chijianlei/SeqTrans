
#include "rclcpp/rclcpp.hpp"
#include <diagnostic_updater/diagnostic_updater.hpp>

//util
#include <swri_nav_util/linear_interpolant.h>

//msgs
#include <nav_msgs/msg/odometry.hpp>
#include <marti_common_msgs/msg/float32_stamped.hpp>
/*
//dynamic reconfig
#include <dynamic_reconfigure/server.h>
#include <speed_controller/GainSchedulePidConfig.h>
*/

namespace du = diagnostic_updater;
namespace nm = nav_msgs::msg;
namespace mcm = marti_common_msgs::msg;

namespace speed_controller
{

class GainSchedulePID : public rclcpp::Node
{
 private:

  rclcpp::TimerBase::SharedPtr init_timer_;
  rclcpp::TimerBase::SharedPtr control_loop_timer_;

  rclcpp::Subscription<nm::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<mcm::Float32Stamped>::SharedPtr speed_command_sub_;

  rclcpp::Publisher<mcm::Float32Stamped>::SharedPtr brake_command_pub_;
  rclcpp::Publisher<mcm::Float32Stamped>::SharedPtr throttle_command_pub_;

  nm::Odometry last_odom_;
  mcm::Float32Stamped last_speed_command_;
  mcm::Float32Stamped last_throttle_command_;
  mcm::Float32Stamped last_brake_command_;

  double odom_timeout_;
  double command_timeout_;

  //parameters
  double stop_speed_threshold_;

  swri_nav_util::LinearInterpolant speed_to_gain_p_;

  double min_integral_contribution_;
  double max_integral_contribution_;
  double i_gain_;

  double feedforward_m_;
  double feedforward_b_;

  double min_gasbrake_command_;
  double max_gasbrake_command_;

  //persistent variables
  bool first_iteration_;
  rclcpp::Time last_command_time_;
  double integral_contribution_;
  bool parameters_updated_;

/* TODO parameter callback replacement
  // Reconfigure
  boost::recursive_mutex reconfig_mutex_;
  typedef dynamic_reconfigure::Server<speed_controller::GainSchedulePidConfig> ReconfigureServer;
  boost::shared_ptr<ReconfigureServer> reconfig_server_;
*/
 public:
  GainSchedulePID(rclcpp::NodeOptions options) :
    Node("gain_schedule_pid", options),
    stop_speed_threshold_(0.1),
    min_integral_contribution_(0.0),
    max_integral_contribution_(0.0),
    i_gain_(0.0),
    feedforward_m_(0.0),
    feedforward_b_(0.0),
    min_gasbrake_command_(0.0),
    max_gasbrake_command_(0.0),
    first_iteration_(true),
    last_command_time_(this->now()),
    integral_contribution_(0.0),
    parameters_updated_(false)
  {
    //declare parameters
    this->declare_parameter("initialization_delay_s", rclcpp::ParameterValue(1.0));
    this->declare_parameter("odom_timeout", rclcpp::ParameterValue(0.5));
    this->declare_parameter("command_timeout", rclcpp::ParameterValue(0.5));

    this->declare_parameter("stop_speed_threshold", rclcpp::ParameterValue(0.0));
    this->declare_parameter("feedforward_m", rclcpp::ParameterValue(0.0));
    this->declare_parameter("feedforward_b", rclcpp::ParameterValue(0.0));

    this->declare_parameter("speed_to_gain_p", rclcpp::ParameterValue("[[0,1000],[0.0,0.0]]"));

    this->declare_parameter("min_integral_contribution", rclcpp::ParameterValue(-10.0));
    this->declare_parameter("max_integral_contribution", rclcpp::ParameterValue(10.0));
    this->declare_parameter("integral_gain", rclcpp::ParameterValue(0.0));
    this->declare_parameter("min_gasbrake_command", rclcpp::ParameterValue(-100.0));
    this->declare_parameter("max_gasbrake_command", rclcpp::ParameterValue(100.0));

    this->declare_parameter("update_rate_hz", rclcpp::ParameterValue(25.0));

    // create parameter callback function
    this->set_on_parameters_set_callback(
      std::bind(
        &GainSchedulePID::dynamicReconfigCallback,
        this,
        std::placeholders::_1
      )
    );

    // call onInit to replicate ROS nodelet functionality
    this->onInit();
  }

  ~GainSchedulePID()
  {
  }

  void onInit()
  {
    // Create a one-shot timer to initialize everything after a brief
    // pause so that ROS has time to connect to rosout so that we
    // don't drop errors/info during initialization.
    double initialization_delay = 1.0;
    initialization_delay = (this->get_parameter("initialization_delay_s")).as_double();
    std::this_thread::sleep_for(
      std::chrono::duration<int, std::milli>(int(1000*initialization_delay))
    );
    this->initialize();
  }

  void initialize()
  {
    // debugging print
    RCLCPP_INFO(this->get_logger(),"initialize()");
    //subscriber
    odom_sub_ = this->create_subscription<nm::Odometry>(
      "odom", 1,
      std::bind(&GainSchedulePID::HandleOdometry, this, std::placeholders::_1)
    );

    speed_command_sub_ = this->create_subscription<mcm::Float32Stamped>(
      "speed_command", 1,
      std::bind(&GainSchedulePID::HandleSpeedCommand, this, std::placeholders::_1)
    );

    //publisher
    throttle_command_pub_ = this->create_publisher<mcm::Float32Stamped>(
      "throttle_command", rclcpp::QoS(1));

    brake_command_pub_ = this->create_publisher<mcm::Float32Stamped>(
      "brake_command", rclcpp::QoS(1));

    odom_timeout_ = (this->get_parameter("odom_timeout")).as_double();
    command_timeout_ = (this->get_parameter("command_timeout")).as_double();

    stop_speed_threshold_ = (this->get_parameter("stop_speed_threshold")).as_double();
    feedforward_m_ = (this->get_parameter("feedforward_m")).as_double();
    feedforward_b_ = (this->get_parameter("feedforward_b")).as_double();

    // Parse gain scheduling
    speed_to_gain_p_.loadFromParameterServer(
      this->get_node_parameters_interface(),
      "speed_to_gain_p");
    if (!speed_to_gain_p_.valid())
    {
      RCLCPP_ERROR(this->get_logger(),
            "Failed to load speed_to_gain_p from parameter server.");
      std::vector<double> zero = {0.0};
      speed_to_gain_p_.set(zero, zero);
    }
    RCLCPP_INFO(this->get_logger(),
            "speed_to_gain_p_ = %s", speed_to_gain_p_.toString().c_str());

    min_integral_contribution_ = (this->get_parameter("min_integral_contribution")).as_double();
    max_integral_contribution_ = (this->get_parameter("max_integral_contribution")).as_double();
    i_gain_ = (this->get_parameter("integral_gain")).as_double();
    min_gasbrake_command_ = (this->get_parameter("min_gasbrake_command")).as_double();
    max_gasbrake_command_ = (this->get_parameter("max_gasbrake_command")).as_double();

    //setup control loop timer
    double update_rate_hz;
    update_rate_hz = (this->get_parameter("update_rate_hz")).as_double();

    control_loop_timer_ = this->create_wall_timer(
      std::chrono::duration<double, std::ratio<1,1>>(1.0 / update_rate_hz),
      std::bind(&GainSchedulePID::ControlLoop,this)
    );
    return;
  }

  rcl_interfaces::msg::SetParametersResult dynamicReconfigCallback(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    parameters_updated_ = true;
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    // print warnings for unsupported parameters
    for(const auto & param : parameters)
    {
      std::string name = param.get_name();
      bool cannot = !name.compare("odom_timeout") ||
        !name.compare("command_timeout") ||
        !name.compare("update_rate_hz");
      if(cannot)
      {
        RCLCPP_WARN(this->get_logger(),
          "Cannot change parameter %s during run",
          name.c_str()
        );
        result.successful = false;
        result.reason = "No support for changing requested parameter during execution";
      }
    }
    return result;
  }

  /** @brief Re-load the parameters when a parameter change is detected. */
  void reloadParameters()
  {
    stop_speed_threshold_ = (this->get_parameter("stop_speed_threshold")).as_double();
    feedforward_m_ = (this->get_parameter("feedforward_m")).as_double();
    feedforward_b_ = (this->get_parameter("feedforward_b")).as_double();

    // Parse gain scheduling
    speed_to_gain_p_.loadFromParameterServer(
      this->get_node_parameters_interface(),
      "speed_to_gain_p");
    if (!speed_to_gain_p_.valid())
    {
      RCLCPP_ERROR(this->get_logger(),
            "Failed to load speed_to_gain_p from parameter server.");
      std::vector<double> zero = {0.0};
      speed_to_gain_p_.set(zero, zero);
    }
    RCLCPP_INFO(this->get_logger(),
            "speed_to_gain_p_ = %s", speed_to_gain_p_.toString().c_str());

    min_integral_contribution_ = (this->get_parameter("min_integral_contribution")).as_double();
    max_integral_contribution_ = (this->get_parameter("max_integral_contribution")).as_double();
    i_gain_ = (this->get_parameter("integral_gain")).as_double();
    min_gasbrake_command_ = (this->get_parameter("min_gasbrake_command")).as_double();
    max_gasbrake_command_ = (this->get_parameter("max_gasbrake_command")).as_double();
  }

  void Shutdown()
  {
  }

  void Disable()
  {
    integral_contribution_ = 0.0;
    first_iteration_ = true;
  }

//  SpeedControlResult Run(const ros::Time &now,
//                         double current_speed,
//                         double desired_speed)
  void ControlLoop()
  {
    if (HasTimedOut())
    {
      return;
    }
    if (parameters_updated_)
    {
      parameters_updated_ = false;
      // reload parameters
      this->reloadParameters();
    }
    rclcpp::Time now = this->now();
    double desired_speed = last_speed_command_.value;
    double current_speed = std::abs(last_odom_.twist.twist.linear.x);
    double gasbrake_percent = 0.0;

    if (desired_speed == 0.0)
    {
      gasbrake_percent = -50.0; //TODO, make this a ros param?
    }
    else
    {
      double speed_error = desired_speed - current_speed;

      double feedforward = FeedForwardTerm(desired_speed);
      double proportional = ProportionalTerm(current_speed, speed_error);
      double integral = IntegralTerm(now, speed_error);

      gasbrake_percent = feedforward + proportional + integral;
      RCLCPP_DEBUG(this->get_logger(),
                "gasbrake %f, feedforward %f, proportional %f, integral %f",
                gasbrake_percent, feedforward, proportional, integral);
      gasbrake_percent = ClampGasBrake(gasbrake_percent);

      first_iteration_ = false;
      last_command_time_ = now;
    }

    //publish
    auto throttle_msg = mcm::Float32Stamped();
    throttle_msg.header.stamp = now;
    throttle_msg.value = std::max(gasbrake_percent, 0.0)/100.0;
    throttle_command_pub_->publish(throttle_msg);

    auto brake_msg = mcm::Float32Stamped();
    brake_msg.header.stamp = now;
    brake_msg.value = std::max(-gasbrake_percent, 0.0)/100.0;
    brake_command_pub_->publish(brake_msg);
  }

 protected:
  double ProportionalTerm(double current_speed, double speed_error)
  {
    double scheduled_gain = speed_to_gain_p_.eval(current_speed);
    RCLCPP_DEBUG(this->get_logger(),
            "cur speed %f, speed error %f, scheduled gain %f, p term %f",
             current_speed, speed_error, scheduled_gain, scheduled_gain * speed_error);
    return scheduled_gain * speed_error;
  }

  double IntegralTerm(const rclcpp::Time &now,
                    double speed_error)
  {
    if (first_iteration_) {
      integral_contribution_ = 0;
    } else {
      double dt = (now - last_command_time_).seconds();
      integral_contribution_ += i_gain_*(speed_error * dt);

      if (integral_contribution_ < min_integral_contribution_)
        integral_contribution_ = min_integral_contribution_;
      else if (integral_contribution_ > max_integral_contribution_)
        integral_contribution_ = max_integral_contribution_;
    }

    return integral_contribution_;
  }

  double FeedForwardTerm(double desired_speed)
  {
    return feedforward_b_ + feedforward_m_ * desired_speed;
  }

  double ClampGasBrake(double gasbrake)
  {
    if (gasbrake < min_gasbrake_command_)
      gasbrake = min_gasbrake_command_;
    if (gasbrake > max_gasbrake_command_)
      gasbrake = max_gasbrake_command_;
    return gasbrake;
  }

  bool HasTimedOut()
  {
    rclcpp::Time now = this->now();

    /// TODO this calculation may not be correct - no ROS2 master clock
    double age_meas = (now - last_odom_.header.stamp).seconds();
    bool odom_timed_out = (age_meas > odom_timeout_);

    if (odom_timed_out)
    {
      RCLCPP_WARN(this->get_logger(),
        "PID: Odometry timed out. Last odom message is %lf "
        "seconds old. Timeout = %lf",
        age_meas,
        odom_timeout_);
    }

    double age_comm = (now - last_speed_command_.header.stamp).seconds();
    bool comm_timed_out = (age_comm > command_timeout_);
    if (comm_timed_out)
    {
      RCLCPP_WARN(this->get_logger(),
          "PID: Speed command has timed out. Last speed "
          "command is %lf seconds old. Timeout = %lf",
          age_comm,
          command_timeout_);
    }

    return (odom_timed_out || comm_timed_out);
  }
  void HandleOdometry(const nm::Odometry::SharedPtr msg)
  {
    last_odom_ = *msg;
  }

  void HandleSpeedCommand(const mcm::Float32Stamped::SharedPtr msg)
  {
    last_speed_command_ = *msg;
    last_speed_command_.value = std::abs(msg->value);
  }
};
}  // namespace speed_controller

// Register nodelet plugin
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(speed_controller::GainSchedulePID)
