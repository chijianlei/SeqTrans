#include <thread>

#include "rclcpp/rclcpp.hpp"
#include <diagnostic_updater/diagnostic_updater.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <marti_common_msgs/msg/float32_stamped.hpp>
#include <marti_common_msgs/msg/bool_stamped.hpp>
#include <marti_dbw_msgs/msg/transmission_feedback.hpp>

#include <speed_controller/longitudinal_dynamics.h>
#include <speed_controller/simple_pid.h>

// Alias type for easier access to DiagnosticStatus enumerations.
typedef diagnostic_msgs::msg::DiagnosticStatus DS;

namespace du = diagnostic_updater;
namespace nm = nav_msgs::msg;
namespace mcm = marti_common_msgs::msg;
namespace mdm = marti_dbw_msgs::msg;

namespace speed_controller
{

class FeedforwardPid : public rclcpp::Node
{
 private:
  std::shared_ptr<du::Updater> diagnostic_updater_;

  rclcpp::TimerBase::SharedPtr init_timer_;
  rclcpp::TimerBase::SharedPtr control_loop_timer_;

  rclcpp::Subscription<nm::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<mcm::Float32Stamped>::SharedPtr speed_command_sub_;
  rclcpp::Subscription<mcm::Float32Stamped>::SharedPtr accel_command_sub_;
  rclcpp::Subscription<mdm::TransmissionFeedback>::SharedPtr transmission_sense_sub_;

  rclcpp::Publisher<mcm::Float32Stamped>::SharedPtr brake_command_pub_;
  rclcpp::Publisher<mcm::Float32Stamped>::SharedPtr throttle_command_pub_;
  rclcpp::Publisher<mcm::Float32Stamped>::SharedPtr pid_measured_filtered_pub_; //DEBUGGING

  //set in subscriber callbacks
  nm::Odometry last_odom_;
  mcm::Float32Stamped last_speed_command_;
  mcm::Float32Stamped last_accel_command_;
  mdm::TransmissionFeedback last_transmission_sense_;

  //params
  double odom_timeout_;
  double command_timeout_;
  double stop_velocity_thresh_;
  bool enable_feedforward_;

  LongitudinalDynamicsModel mdl_;
  PidConfig pid_config_;
  SimplePid pid_;
  double stop_brake_;

  bool parameters_initialized_;
  bool parameters_updated_; /**< Set to true by the parameter callback */

  //persist between updates
  bool is_stopped_;

 public:
  FeedforwardPid(rclcpp::NodeOptions options):
    Node("feedforward_pid", options),
    odom_timeout_(0.0),
    command_timeout_(0.0),
    stop_velocity_thresh_(0.01),
    enable_feedforward_(true),
    parameters_initialized_(false),
    is_stopped_(false),
    parameters_updated_(false),
    mdl_(this->get_logger().get_child("longitudinal_dynamics")),
    pid_config_(this->get_logger().get_child("pid_config")),
    pid_(this->get_logger().get_child("pid"),this->now())
  {
    this->declare_parameter("initialization_delay_s", rclcpp::ParameterValue(1.0));
    // parameters for longitudinal dynamics member
    this->declare_parameter("delay", rclcpp::ParameterValue(0.0));
    this->declare_parameter("throttle_deadband", rclcpp::ParameterValue(0.0));
    this->declare_parameter("throttle_gain", rclcpp::ParameterValue(0.0));
    this->declare_parameter("idle_power", rclcpp::ParameterValue(0.0));
    this->declare_parameter("speed_offset", rclcpp::ParameterValue(0.0));
    this->declare_parameter("peak_force", rclcpp::ParameterValue(1000.0));
    this->declare_parameter("brake_deadband", rclcpp::ParameterValue(0.0));
    this->declare_parameter("brake_gain", rclcpp::ParameterValue(0.0));
    this->declare_parameter("brake_gain2", rclcpp::ParameterValue(0.0));
    this->declare_parameter("resistance_coeff_1", rclcpp::ParameterValue(0.0));
    this->declare_parameter("resistance_coeff_v", rclcpp::ParameterValue(0.0));
    this->declare_parameter("resistance_coeff_v2", rclcpp::ParameterValue(0.0));
    this->declare_parameter("grav_gain", rclcpp::ParameterValue(0.0));
    // parameters used by this object
    this->declare_parameter("odom_timeout", rclcpp::ParameterValue(0.1));
    this->declare_parameter("command_timeout", rclcpp::ParameterValue(0.1));
    this->declare_parameter("stop_velocity_threshold", rclcpp::ParameterValue(0.01));
    this->declare_parameter("enable_feedforward", rclcpp::ParameterValue(true));

    this->declare_parameter("kp", rclcpp::ParameterValue(0.0));
    this->declare_parameter("ki", rclcpp::ParameterValue(0.0));
    this->declare_parameter("min_integral", rclcpp::ParameterValue(0.0));
    this->declare_parameter("max_integral", rclcpp::ParameterValue(0.0));
    this->declare_parameter("measured_filter_cut_off_hz", rclcpp::ParameterValue(0.0));

    this->declare_parameter("stop_brake", rclcpp::ParameterValue(0.5));
    this->declare_parameter("update_rate_hz", rclcpp::ParameterValue(20.0));

    // set up the parameter callback function
    this->set_on_parameters_set_callback(std::bind(&FeedforwardPid::ReconfigCb,this,std::placeholders::_1));

    this->onInit();
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
    RCLCPP_INFO(this->get_logger(),"initialize()");/// debugging print
    //Initialize subscribers
    odom_sub_ = this->create_subscription<nm::Odometry>(
      "odom", 1,
      std::bind(&FeedforwardPid::HandleOdometry, this, std::placeholders::_1)
    );

    speed_command_sub_ = this->create_subscription<mcm::Float32Stamped>(
      "speed_command", 1,
      std::bind(&FeedforwardPid::HandleSpeedCommand, this, std::placeholders::_1)
    );

    accel_command_sub_ = this->create_subscription<mcm::Float32Stamped>(
      "acceleration_command", 1,
      std::bind(&FeedforwardPid::HandleAccelerationCommand, this, std::placeholders::_1)
    );

    transmission_sense_sub_ = this->create_subscription<mdm::TransmissionFeedback>(
      "transmission_sense", 1,
      std::bind(&FeedforwardPid::HandleTransmissionSense, this, std::placeholders::_1)
    );

    //Initialize publishers
    throttle_command_pub_ = this->create_publisher<mcm::Float32Stamped>(
      "throttle_command", rclcpp::QoS(1));

    brake_command_pub_ = this->create_publisher<mcm::Float32Stamped>(
      "brake_command", rclcpp::QoS(1));

    pid_measured_filtered_pub_ = this->create_publisher<mcm::Float32Stamped>(
      "pid_measured_filtered", rclcpp::QoS(1)); //DEBUGGING

    //parameters
    odom_timeout_ = (this->get_parameter("odom_timeout")).as_double();
    command_timeout_ = (this->get_parameter("command_timeout")).as_double();
    stop_velocity_thresh_ = (this->get_parameter("stop_velocity_threshold")).as_double();
    enable_feedforward_ = (this->get_parameter("enable_feedforward")).as_bool();

    // Initialize LongitudinalDynamicsModel
    mdl_.getParams(this->get_node_parameters_interface());

    //Initialize PID
    double kp, ki, min_i, max_i, meas_hz;
    kp = (this->get_parameter("kp")).as_double();
    ki = (this->get_parameter("ki")).as_double();
    min_i = (this->get_parameter("min_integral")).as_double();
    max_i = (this->get_parameter("max_integral")).as_double();
    meas_hz = (this->get_parameter("measured_filter_cut_off_hz")).as_double();

    pid_config_.SetGains(kp, ki, 0.0);
    pid_config_.SetLimits(min_i, max_i, 0.0, 0.0);
    pid_config_.SetFilterCutOffHz(meas_hz, 0.0);
    pid_config_.PrintConfig();
    pid_.SetConfig(pid_config_);

    stop_brake_ = (this->get_parameter("stop_brake")).as_double();

    //initialize diagnostic updater
    diagnostic_updater_ = std::make_shared<du::Updater>(this->create_sub_node("feedforward_pid_diagnostics"));
    diagnostic_updater_->setHardwareID("none");
    diagnostic_updater_->add(
      "Feedforward PID", this,
      &FeedforwardPid::updateDiagnostics);

    //Set flag
    parameters_initialized_ = true;

    //Initialize update timer
    double update_rate_hz;
    update_rate_hz = (this->get_parameter("update_rate_hz")).as_double();

    control_loop_timer_ = this->create_wall_timer(
      std::chrono::duration<double, std::ratio<1,1>>(1.0 / update_rate_hz),
      std::bind(&FeedforwardPid::ControlLoop,this)
    );
  }

  rcl_interfaces::msg::SetParametersResult ReconfigCb(const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    /*
    if (!parameters_initialized_)
    {
      result.successful = false;
      result.reason = "Parameters not initialized";
      return result;
    }
    */
    // else, permit initialization
    parameters_updated_ = true;
    return result;
  }

  void updateDiagnostics(du::DiagnosticStatusWrapper& status)
  {
    status.summary(DS::OK, "No errors reported.");
  }

  void ControlLoop()
  {
    if (parameters_updated_)
    {
      // reset flag
      parameters_updated_ = false;
      // reload remaining parameters
      enable_feedforward_ = (this->get_parameter("enable_feedforward")).as_bool();
      mdl_.getParams(this->get_node_parameters_interface());

      odom_timeout_ = (this->get_parameter("odom_timeout")).as_double();
      command_timeout_ = (this->get_parameter("command_timeout")).as_double();
      stop_velocity_thresh_ = (this->get_parameter("stop_velocity_threshold")).as_double();
      double kp, ki, min_i, max_i, meas_hz;
      kp = (this->get_parameter("kp")).as_double();
      ki = (this->get_parameter("ki")).as_double();
      min_i = (this->get_parameter("min_integral")).as_double();
      max_i = (this->get_parameter("max_integral")).as_double();
      meas_hz = (this->get_parameter("measured_filter_cut_off_hz")).as_double();

      pid_config_.SetGains(kp, ki, 0.0);
      pid_config_.SetLimits(min_i, max_i, 0.0, 0.0);
      pid_config_.SetFilterCutOffHz(meas_hz, 0.0);
      pid_config_.PrintConfig();
      pid_.SetConfig(pid_config_);

      stop_brake_ = (this->get_parameter("stop_brake")).as_double();
    }

    if (HasTimedOut())
    {
      RCLCPP_WARN(this->get_logger(), "Subscriber has timed out.");
      return;
    }

    rclcpp::Time now = this->now();

    // Handle stop/stopping condition first
    if (last_speed_command_.value == 0.0)
    {
      if (!is_stopped_ && pid_.GetMeasuredFiltered() < stop_velocity_thresh_)
      {
        is_stopped_ = true; //both commanded and measured speed are zero
      }
    }
    else //commanded speed is nonzero
    {
      is_stopped_ = false;
    }

    double throttle_cmd = 0.0;
    double brake_cmd = 0.0;

    if (is_stopped_)
    {
      // Hold the brake:
      brake_cmd = stop_brake_;
      pid_.Reset();
    }
    else
    {
      //TODO, Need to filter the vehicle pitch to get ground pitch
//      double pitch = tf::getYaw(last_odom_.pose.pose.orientation);
      double pitch = 0.0; //DEBUGGING

      double gasbrake_ff = 0.0; //range -1 to 1
      if (enable_feedforward_)
      {
        //set acceleration setpoint to zero when stop is commanded
        if (last_speed_command_.value == 0.0)
        {
          last_accel_command_.value = 0.0;
        }

        bool in_reverse = last_transmission_sense_.reverse;
        double velocity = last_speed_command_.value * (in_reverse ? -1.0 : 1.0);
        double acceleration = last_accel_command_.value * (in_reverse ? -1.0 : 1.0);

        if (!mdl_.calcThrottleBrake(acceleration, velocity, pitch, in_reverse, gasbrake_ff))
        {
          RCLCPP_WARN(this->get_logger(),"Failed call to calcThrottleBrake(acceleration = %lf, velocity = %lf, pitch = %lf, in_reverse = %d)",
              acceleration, velocity, pitch, in_reverse); //DEBUGGING
          gasbrake_ff = 0.0; //should already be zero
        }
      }

      //get raw throttle & brake commands from pid
      double gasbrake_pid = pid_.Update(now);
      double gasbrake = gasbrake_ff + gasbrake_pid;

      RCLCPP_DEBUG(this->get_logger(),"cmd speed = %lf, acceleration = %lf, meas speed = %lf, gasbrake = %lf (feedforward = %lf, pid = %lf)",
          last_speed_command_.value, last_accel_command_.value,
          pid_.GetMeasuredFiltered(),
          gasbrake, gasbrake_ff, gasbrake_pid); //DEBUGGING

      gasbrake = std::max(-1.0, std::min(gasbrake, 1.0)); //Clamp output

      if (gasbrake > 0.0)
      {
        throttle_cmd = gasbrake;
      }
      else if (gasbrake < 0.0)
      {
        brake_cmd = -gasbrake;
      }
    }

//    now = ros::Time::now();
    throttle_command_pub_->publish(*makeFloat32StampedPtr(now, throttle_cmd));
    brake_command_pub_->publish(*makeFloat32StampedPtr(now, brake_cmd));
    pid_measured_filtered_pub_->publish(*makeFloat32StampedPtr(now, pid_.GetMeasuredFiltered())); //DEBUGGING
  }

  /// @brief Check if subscribers have timed out
  ///
  /// @details this function is a stub; the swri_roscpp
  /// version has some internal diagnostics that run here
  bool HasTimedOut()
  {
    bool timeout_error = false;

    return timeout_error;
  }

  void HandleOdometry(const nm::Odometry::SharedPtr msg)
  {
    last_odom_ = *msg;
    double speed = std::abs(msg->twist.twist.linear.x);
    //TODO, instead of taking absolute value, would it be better to
    //subscribe to transmission feedback and negate if in reverse.
    //that way speed controller can tell if moving in wrong direction.
    pid_.LoadMeasured(speed);
  }

  void HandleSpeedCommand(const mcm::Float32Stamped::SharedPtr msg)
  {
    last_speed_command_ = *msg;
    last_speed_command_.value = std::abs(msg->value);
    pid_.LoadCommanded(last_speed_command_.value);
  }

  //TODO, better to combine speed and acceleration into a single msg
  //because they should always be specified as a pair.
  //Otherwise there is a chance of momentarily receiving speed and
  //acceleration from different sources.
  //(e.g. speed from teleop but acceleration from path_following_controller)
  void HandleAccelerationCommand(const mcm::Float32Stamped::SharedPtr msg)
  {
    last_accel_command_ = *msg;
  }

  void HandleTransmissionSense(const mdm::TransmissionFeedback::SharedPtr msg)
  {
    last_transmission_sense_ = *msg;
  }

};  // class FeedforwardPid

/// I don't know what this is or if it needs to be converted to ROS2
/*
boost::shared_ptr<nodelet::Nodelet> createFeedForwardPid()
{
  return boost::make_shared<FeedforwardPid>();
}
*/
}  // namespace

// Register nodelet plugin
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(speed_controller::FeedforwardPid)
