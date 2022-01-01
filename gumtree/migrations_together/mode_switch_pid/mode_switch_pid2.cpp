#include <thread>

#include "rclcpp/rclcpp.hpp"
#include <diagnostic_updater/diagnostic_updater.hpp>

//msgs
#include <nav_msgs/msg/odometry.hpp>
#include <marti_common_msgs/msg/float32_stamped.hpp>
//dynamic reconfig not implemented

namespace du = diagnostic_updater;
namespace nm = nav_msgs::msg;
namespace mcm = marti_common_msgs::msg;

namespace speed_controller
{

class ModeSwitchPid : public rclcpp::Node
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
  double brake_error_threshold_;
  int brake_saturated_threshold_;
  double brake_kp_;
  double brake_ki_;
  double brake_integral_;
  double max_brake_integral_;
  double min_brake_integral_;
  rclcpp::Time brake_last_time_;
  int brake_saturated_count_;

  double throttle_error_threshold_;
  int throttle_saturated_threshold_;
  double throttle_kp_;
  double throttle_ki_;
  double throttle_integral_;
  double max_throttle_integral_;
  double min_throttle_integral_;
  rclcpp::Time throttle_last_time_;
  int throttle_saturated_count_;

  enum SpeedControllerState {
    STATE_STOP,
    STATE_APPLY_THROTTLE,
    STATE_APPLY_BRAKE
  };

  SpeedControllerState state_;
  SpeedControllerState last_state_;

 public:
  ModeSwitchPid(rclcpp::NodeOptions options)
    :
    Node("mode_switch_pid", options),
    brake_error_threshold_(0.0),
    brake_saturated_threshold_(0.0),
    brake_kp_(0.0),
    brake_ki_(0.0),
    brake_integral_(0.0),
    max_brake_integral_(0.0),
    min_brake_integral_(0.0),
    brake_last_time_(this->now()),
    brake_saturated_count_(0),
    throttle_error_threshold_(0.0),
    throttle_saturated_threshold_(0.0),
    throttle_kp_(0.0),
    throttle_ki_(0.0),
    throttle_integral_(0.0),
    max_throttle_integral_(0.0),
    min_throttle_integral_(0.0),
    throttle_last_time_(0.0),
    throttle_saturated_count_(0),
    state_(STATE_STOP),
    last_state_(STATE_STOP)
  {
    // declare parameters
    this->declare_parameter("initialization_delay_s",rclcpp::ParameterValue(1.0));

    this->declare_parameter("odom_timeout",rclcpp::ParameterValue(0.5));
    this->declare_parameter("command_timeout",rclcpp::ParameterValue(0.5));

    this->declare_parameter("brake_error_threshold",rclcpp::ParameterValue(0.0));
    this->declare_parameter("brake_saturated_threshold",rclcpp::ParameterValue(0.0));
    this->declare_parameter("brake_kp",rclcpp::ParameterValue(0.0));
    this->declare_parameter("brake_ki",rclcpp::ParameterValue(0.0));
    this->declare_parameter("max_brake_integral",rclcpp::ParameterValue(0.0));
    this->declare_parameter("min_brake_integral",rclcpp::ParameterValue(0.0));
    this->declare_parameter("throttle_error_threshold",rclcpp::ParameterValue(0.0));
    this->declare_parameter("throttle_saturated_threshold",rclcpp::ParameterValue(0.0));
    this->declare_parameter("throttle_kp",rclcpp::ParameterValue(0.0));
    this->declare_parameter("throttle_ki",rclcpp::ParameterValue(0.0));
    this->declare_parameter("max_throttle_integral",rclcpp::ParameterValue(0.0));
    this->declare_parameter("min_throttle_integral",rclcpp::ParameterValue(0.0));

    this->declare_parameter("update_rate_hz",rclcpp::ParameterValue(25.0));
    // wrap ROS1 nodelet initializer
    this->onInit();
  }

  ~ModeSwitchPid()
  {
  }

  void onInit()
  {
    // Create a one-shot timer to initialize everything after a brief
    // pause so that ROS has time to connect to rosout so that we
    // don't drop errors/info during initialization.
    double initialization_delay = 1.0;
    initialization_delay = (this->get_parameter("initialization_delay_s")).as_double();
    // use sleep_for to replace oneshot ROS timer
    std::this_thread::sleep_for(
      std::chrono::duration<int, std::milli>(int(1000*initialization_delay))
    );
    this->initialize();
  }

  void initialize()
  {
    //subscriber
    odom_sub_ = this->create_subscription<nm::Odometry>(
      "odom", 1,
      std::bind(&ModeSwitchPid::HandleOdometry, this, std::placeholders::_1)
    );

    speed_command_sub_ = this->create_subscription<mcm::Float32Stamped>(
      "speed_command", 1,
      std::bind(&ModeSwitchPid::HandleSpeedCommand, this, std::placeholders::_1)
    );

    //publisher
    throttle_command_pub_ = this->create_publisher<mcm::Float32Stamped>(
      "throttle_command", rclcpp::QoS(1));

    brake_command_pub_ = this->create_publisher<mcm::Float32Stamped>(
      "brake_command", rclcpp::QoS(1));

    odom_timeout_ = (this->get_parameter("odom_timeout")).as_double();
    command_timeout_ = (this->get_parameter("command_timeout")).as_double();

    //parameters
    brake_error_threshold_ = (this->get_parameter("brake_error_threshold")).as_double();
    brake_saturated_threshold_ = (this->get_parameter("brake_saturated_threshold")).as_double();
    brake_kp_ = (this->get_parameter("brake_kp")).as_double();
    brake_ki_ = (this->get_parameter("brake_ki")).as_double();
    max_brake_integral_ = (this->get_parameter("max_brake_integral")).as_double();
    min_brake_integral_ = (this->get_parameter("min_brake_integral")).as_double();
    throttle_error_threshold_ = (this->get_parameter("throttle_error_threshold")).as_double();
    throttle_saturated_threshold_ = (this->get_parameter("throttle_saturated_threshold")).as_double();
    throttle_kp_ = (this->get_parameter("throttle_kp")).as_double();
    throttle_ki_ = (this->get_parameter("throttle_ki")).as_double();
    max_throttle_integral_ = (this->get_parameter("max_throttle_integral")).as_double();
    min_throttle_integral_ = (this->get_parameter("min_throttle_integral")).as_double();

    //setup control loop timer
    double update_rate_hz = (this->get_parameter("update_rate_hz")).as_double();

    control_loop_timer_ = this->create_wall_timer(
      std::chrono::duration<double,std::ratio<1,1>>(1.0/update_rate_hz),
      std::bind(&ModeSwitchPid::ControlLoop,this)
    );

    return;
  }

  void Shutdown()
  {
  }

  void Disable()
  {
    state_ = STATE_STOP;
  }

  void ControlLoop()
  {
    if (HasTimedOut())
    {
      return;
    }
    rclcpp::Time now = this->now();
    double desired_speed = last_speed_command_.value;
    double current_speed = std::abs(last_odom_.twist.twist.linear.x);
    double gasbrake_percent = -50.0;

    if (desired_speed == 0.0)
      state_ = STATE_STOP;

    SpeedControllerState temp_state = state_;

    if (state_ == STATE_STOP) {
      gasbrake_percent = RunStop(desired_speed);
    } else if (state_ == STATE_APPLY_THROTTLE) {
      gasbrake_percent = RunThrottle(now, desired_speed, current_speed);
    } else if (state_ == STATE_APPLY_BRAKE) {
      gasbrake_percent = RunBrake(now, desired_speed, current_speed);
    }

    last_state_ = temp_state;

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
  double RunStop(double desired)
  {
    if (desired != 0.0) {
      state_ = STATE_APPLY_BRAKE;
    }

    return -50.0;
  }

  double RunThrottle(const rclcpp::Time &now, double desired, double measured)
  {
    if (last_state_ != STATE_APPLY_THROTTLE) {
      throttle_integral_ = 0.0;
      throttle_last_time_ = now;
      throttle_saturated_count_ = 0;
    }

    double error = desired - measured;

    if (error < -throttle_error_threshold_) {
      // Too far above the desired speed, switch to brake.
      state_ = STATE_APPLY_BRAKE;

      return 0.0;
    }

    double dt = (now - throttle_last_time_).seconds();
    throttle_last_time_ = now;

    throttle_integral_ += throttle_ki_*error*dt;
    throttle_integral_ = Clamp(min_throttle_integral_,
                               throttle_integral_,
                               max_throttle_integral_);

    double throttle = throttle_kp_*error + throttle_integral_;

    if (throttle < 0.0) {
      throttle_saturated_count_++;
    } else if (throttle_saturated_count_ > 0) {
      throttle_saturated_count_--;
    }

    if (throttle_saturated_count_ > throttle_saturated_threshold_)
    {
      state_ = STATE_APPLY_BRAKE;
      throttle = 0.0;
    }

    throttle = Clamp(0.0, throttle, 100.0);

    return throttle;
  }

  double RunBrake(const rclcpp::Time &now, double desired, double measured)
  {
    if (last_state_ != STATE_APPLY_BRAKE) {
      brake_integral_ = 0.0;
      brake_last_time_ = now;
      brake_saturated_count_ = 0;
    }

    double error = desired - measured;
    if (error > brake_error_threshold_) {
      // Too far below the desired speed, switch to throttle.
      state_ = STATE_APPLY_THROTTLE;

      return 0.0;
    }

    double dt = (now - brake_last_time_).seconds();
    brake_last_time_ = now;

    brake_integral_ += brake_ki_*error*dt;
    brake_integral_ = Clamp(min_brake_integral_,
                            brake_integral_,
                            max_brake_integral_);

    double brake = brake_kp_*error + brake_integral_;

    if (brake > 0.0) {
      brake_saturated_count_++;
    } else if (brake_saturated_count_ > 0) {
      brake_saturated_count_--;
    }

    if (brake_saturated_count_ > brake_saturated_threshold_)
    {
      state_ = STATE_APPLY_THROTTLE;
      brake = 0.0;
    }

    brake = Clamp(-100.0, brake, 0.0);

    return brake;
  }

  double Clamp(double min, double value, double max)
  {
    if (value < min)
      return min;
    if (value > max)
      return max;
    return value;
  }

  bool HasTimedOut()
  {
    rclcpp::Time now = this->now();

    /// TODO this will not work for ROS 2 is the odometry publisher is on a different computer
    double age_meas = (now - last_odom_.header.stamp).seconds();
    bool odom_timed_out = (age_meas > odom_timeout_);

    if (odom_timed_out)
    {
      RCLCPP_WARN(this->get_logger(), "PID: Odometry timed out. Last odom message is %lf "
         "seconds old. Timeout = %lf",
               age_meas,
         odom_timeout_);
    }

    /// TODO this will not work for ROS 2 is the odometry publisher is on a different computer
    double age_comm = (now - last_speed_command_.header.stamp).seconds();
    bool comm_timed_out = (age_comm > command_timeout_);
    if (comm_timed_out)
    {
      RCLCPP_WARN(this->get_logger(), "PID: Speed command has timed out. Last speed "
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
RCLCPP_COMPONENTS_REGISTER_NODE(speed_controller::ModeSwitchPid)
