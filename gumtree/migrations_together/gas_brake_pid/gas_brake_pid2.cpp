#include <thread>

#include "rclcpp/rclcpp.hpp"
//#include <nodelet/nodelet.h>
#include <diagnostic_updater/diagnostic_updater.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <marti_dbw_msgs/msg/transmission_feedback.hpp>
#include <marti_common_msgs/msg/float32_stamped.hpp>
#include <marti_common_msgs/msg/bool_stamped.hpp>

#include <speed_controller/simple_pid.h>

// for parameter callback
#include "rcl_interfaces/msg/set_parameters_result.hpp"

// Alias type for easier access to DiagnosticStatus enumerations.
typedef diagnostic_msgs::msg::DiagnosticStatus DS;

namespace du = diagnostic_updater;
namespace nm = nav_msgs::msg;
namespace mdm = marti_dbw_msgs::msg;
namespace mcm = marti_common_msgs::msg;

namespace speed_controller
{

class GasBrakePid : public rclcpp::Node
{
 private:
  std::shared_ptr<du::Updater> diagnostic_updater_;

  rclcpp::TimerBase::SharedPtr init_timer_;
  rclcpp::TimerBase::SharedPtr control_loop_timer_;

  rclcpp::Subscription<nm::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<mcm::Float32Stamped>::SharedPtr speed_command_sub_;

  rclcpp::Publisher<mcm::Float32Stamped>::SharedPtr brake_command_pub_;
  rclcpp::Publisher<mcm::Float32Stamped>::SharedPtr throttle_command_pub_;
  rclcpp::Publisher<mcm::Float32Stamped>::SharedPtr pid_measured_filtered_pub_; //DEBUGGING
  rclcpp::Publisher<mcm::Float32Stamped>::SharedPtr pid_derr_filtered_pub_; //DEBUGGING

  //set in subscriber callbacks
  nm::Odometry last_odom_;
  mcm::Float32Stamped last_speed_command_;

  //params
  double odom_timeout_;
  double command_timeout_;
  double stop_velocity_thresh_;

  bool verbose_;

  ControlMapConfig control_map_config_;
  PidConfig pid_config_;
  SimplePid pid_;
  double max_throttle_rate_;
  double max_brake_rate_;
  double update_rate_hz_; /**< Desired control_loop execution rate in Hz */

  bool parameters_initialized_;

  //persist between updates
  bool is_stopped_;
  double last_throttle_command_;
  double last_brake_command_;

  bool parameters_updated_; /**< Set to true in the parameter callback */

 public:

  void onInit()
  {
    // Create a one-second delay to mimic the behavior of the ROS1 nodelet
    double initialization_delay = 1.0;
    initialization_delay = (this->get_parameter("initialization_delay_s")).as_double();
    std::this_thread::sleep_for(
      std::chrono::duration<int, std::milli>(int(1000*initialization_delay))
    );
    this->initialize();
  }

  void initialize()
  {
    odom_sub_ = this->create_subscription<nm::Odometry>(
      "odom", 1,
      std::bind(&GasBrakePid::HandleOdometry, this, std::placeholders::_1)
    );

    speed_command_sub_ = this->create_subscription<mcm::Float32Stamped>(
      "speed_command", 1,
      std::bind(&GasBrakePid::HandleSpeedCommand, this, std::placeholders::_1)
    );

    throttle_command_pub_ = this->create_publisher<mcm::Float32Stamped>(
      "throttle_command", rclcpp::QoS(1));

    brake_command_pub_ = this->create_publisher<mcm::Float32Stamped>(
      "brake_command", rclcpp::QoS(1));

    //DEBUGGING
    pid_measured_filtered_pub_ = this->create_publisher<mcm::Float32Stamped>(
      "pid_measured_filtered", rclcpp::QoS(1));
    pid_derr_filtered_pub_ = this->create_publisher<mcm::Float32Stamped>(
      "pid_derr_filtered", rclcpp::QoS(1));

    // TODO(kkozak): Publish is_stopped message

    verbose_ = (this->get_parameter("verbose")).as_bool();

    odom_timeout_ = (this->get_parameter("odom_timeout")).as_double();
    command_timeout_ = (this->get_parameter("command_timeout")).as_double();
    stop_velocity_thresh_ = (this->get_parameter("stop_velocity_threshold")).as_double();

    max_throttle_rate_ = (this->get_parameter("max_throttle_rate")).as_double();
    max_brake_rate_ = (this->get_parameter("max_brake_rate")).as_double();

    // Initialize PID
    InitializePid();

    // Get control mapping parameters
    InitializeControlMap();

    update_rate_hz_ = (this->get_parameter("update_rate_hz")).as_double();

    control_loop_timer_ = this->create_wall_timer(
      std::chrono::duration<double, std::ratio<1,1>>(1.0 / update_rate_hz_),
      std::bind(&GasBrakePid::ControlLoop,this)
    );

    //diagnostic_updater_ = std::make_shared<du::Updater>(this->make_shared(this->get_name()));
    diagnostic_updater_ = std::make_shared<du::Updater>(this->create_sub_node("gas_brake_pid_diagnostics"));
    diagnostic_updater_->setHardwareID("none");
    diagnostic_updater_->add(
      "Gas\\Brake PID", this,
      &GasBrakePid::updateDiagnostics);

    LoadParametersToReconfig();
  }

  void LoadParametersToReconfig()
  {
    // this functionality is no longer used in ROS2, this function is
    // a stub to re-create the behavior of the ROS1 node.
    RCLCPP_INFO(this->get_logger(), "Loading initial parameters to reconfig");

    pid_config_.PrintConfig();

    control_map_config_.PrintConfig();
    RCLCPP_INFO(this->get_logger(), "Setting throttle and brake limits:\n"
             "max_throttle_rate (0.0 means none): %lf\n"
             "max_brake_rate (0.0 means none): %lf",
             max_throttle_rate_,
             max_brake_rate_);

    parameters_initialized_ = true;
  }

  rcl_interfaces::msg::SetParametersResult ReconfigCb(const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    // prevent return if parameter uninitialized
    if (!parameters_initialized_)
    {
      result.successful = false;
      result.reason = "Parameters not initialized";
      return result;
    }
    // else, permit initialization
    parameters_updated_ = true;
    return result;
  }


  void InitializePid()
  {
    double kp;
    double ki;
    double kd;
    double kp_stop;
    double ki_stop;
    double kd_stop;
    double min_i;
    double max_i;
    double min_d;
    double max_d;
    double ff_ratio;
    double meas_hz;
    double derr_hz;

    kp = (this->get_parameter("kp")).as_double();
    ki = (this->get_parameter("ki")).as_double();
    kd = (this->get_parameter("kd")).as_double();
    kp_stop = (this->get_parameter("kp_stop")).as_double();
    ki_stop = (this->get_parameter("ki_stop")).as_double();
    kd_stop = (this->get_parameter("kd_stop")).as_double();
    min_i = (this->get_parameter("min_integral")).as_double();
    max_i = (this->get_parameter("max_integral")).as_double();
    min_d = (this->get_parameter("min_derivative")).as_double();
    max_d = (this->get_parameter("max_derivative")).as_double();
    ff_ratio = (this->get_parameter("feedforward_vs_speed_ratio")).as_double();
    meas_hz = (this->get_parameter("measured_filter_cut_off_hz")).as_double();
    derr_hz = (this->get_parameter("derr_filter_cut_off_hz")).as_double();

    pid_config_.SetGains(kp, ki, kd);
    pid_config_.SetGainsStop(kp_stop, ki_stop, kd_stop);
    pid_config_.SetLimits(min_i, max_i, min_d, max_d);
    pid_config_.SetFeedforwardVsSpeedRatio(ff_ratio);
    pid_config_.SetFilterCutOffHz(meas_hz, derr_hz);

    if (verbose_)
    {
      pid_config_.PrintConfig();
    }

    pid_.SetConfig(pid_config_);

  }

  void InitializeControlMap()
  {
    double min_brake;
    double max_brake;
    double brake_stop;
    double min_throttle;
    double max_throttle;
    double min_stop_brake;

    min_brake = (this->get_parameter("min_brake")).as_double();
    max_brake = (this->get_parameter("max_brake")).as_double();
    brake_stop = (this->get_parameter("brake_stop")).as_double();
    min_throttle = (this->get_parameter("min_throttle")).as_double();
    max_throttle = (this->get_parameter("max_throttle")).as_double();
    min_stop_brake = (this->get_parameter("min_stop_brake")).as_double();

    control_map_config_.SetMap(min_brake, max_brake, brake_stop,
                               min_throttle, max_throttle, min_stop_brake);

    if (verbose_)
    {
      control_map_config_.PrintConfig();
    }
  }

  void updateDiagnostics(du::DiagnosticStatusWrapper& status)
  {
    status.summary(DS::OK, "No errors reported.");
  }

  void ControlLoop()
  {
    // This is the main Control Loop
    rclcpp::Time now = this->now();
    diagnostic_updater_->update();

    /// check if parameters updated - this replaces a direct callback after
    /// parameters are changed
    if(parameters_updated_)
    {
      // re-set flag
      parameters_updated_ = false;
      // re-load parameters

      verbose_ = (this->get_parameter("verbose")).as_bool();

      odom_timeout_ = (this->get_parameter("odom_timeout")).as_double();
      command_timeout_ = (this->get_parameter("command_timeout")).as_double();
      stop_velocity_thresh_ = (this->get_parameter("stop_velocity_threshold")).as_double();

      max_throttle_rate_ = (this->get_parameter("max_throttle_rate")).as_double();
      max_brake_rate_ = (this->get_parameter("max_brake_rate")).as_double();

      // Initialize PID
      InitializePid();

      // Get control mapping parameters
      InitializeControlMap();

      update_rate_hz_ = (this->get_parameter("update_rate_hz")).as_double();
    }


    bool timed_out = HasTimedOut();
    if (timed_out)
    {
      RCLCPP_WARN(this->get_logger(), "Speed Control input has timed out. "
           "Control output is suspended.");
      return;
    }

    // Handle stop/stopping condition first
    if (last_speed_command_.value == 0.0)
    {
      // Check to see if currently stopped
      if (!is_stopped_)
      {
        // Check to see if the speed is low enough to be considered stopped
        if (pid_.GetMeasuredFiltered() < stop_velocity_thresh_) //TODO, check this
        {
          is_stopped_ = true;
        }
      }
      else
      {
        // Stay in is_stopped_ = true state;
      }
    }
    else
    {
      is_stopped_ = false;
    }

    double throttle_cmd = 0.0;
    double brake_cmd = 0.0;

    if (is_stopped_)
    {
      // Hold the brake:
      brake_cmd = control_map_config_.GetBrakeStop();
      // Don't pass this through the control map;
      pid_.Reset(); //TODO, necessary?
    }
    else
    {
      now = this->now();

      //get raw throttle & brake commands from pid
      double output_command = ClampOutput(pid_.Update(now));
      if (output_command > 0.0)
      {
        throttle_cmd = output_command;
      }
      else if (output_command < 0.0)
      {
        brake_cmd = -output_command;
      }

      // Map the throttle brake commands to appropriate range
      bool cmd_stop = last_speed_command_.value == 0.0; // Debugging a min stop brake command
      throttle_cmd = control_map_config_.MapThrottle(throttle_cmd);
      brake_cmd = control_map_config_.MapBrake(brake_cmd, cmd_stop);

      // Apply throttle or brake rate limits
      double dt = 1.0 / update_rate_hz_;
      if (max_throttle_rate_ > 0.0)
      {
        throttle_cmd = std::min(throttle_cmd, last_throttle_command_ + max_throttle_rate_*dt);
      }

      if (max_brake_rate_ > 0.0)
      {
        brake_cmd = std::min(brake_cmd, last_brake_command_ + max_brake_rate_*dt);
      }
    }

    rclcpp::Time cur_time = this->now();

    // hacky call that reuses existing code
    throttle_command_pub_->publish(*makeFloat32StampedPtr(now, throttle_cmd));
    brake_command_pub_->publish(*makeFloat32StampedPtr(now, brake_cmd));

    // Save throttle & brake commands to limit rates on next update
    last_throttle_command_ = throttle_cmd;
    last_brake_command_ = brake_cmd;

    //DEBUGGING
    pid_measured_filtered_pub_->publish(*makeFloat32StampedPtr(now, pid_.GetMeasuredFiltered()));
    pid_derr_filtered_pub_->publish(*makeFloat32StampedPtr(now, pid_.GetDerrFiltered()));
  }

  double ClampOutput(double val)
  {
    if (val > 1.0)
    {
      val = 1.0;
    }
    else if (val < -1.0)
    {
      val = -1.0;
    }
    return val;
  }

  bool HasTimedOut()
  {
    rclcpp::Time now = this->now();

    double age_meas = (now - last_odom_.header.stamp).seconds();
    bool odom_timed_out = (age_meas > odom_timeout_);

    if (odom_timed_out)
    {
      RCLCPP_WARN(this->get_logger(),"PID: Odometry timed out. "
        "Last odom message is %lf seconds old. Timeout = %lf",
               age_meas,
	       odom_timeout_);
    }

    double age_comm = (now - last_speed_command_.header.stamp).seconds();
    bool comm_timed_out = (age_comm > command_timeout_);
    if (comm_timed_out)
    {
      RCLCPP_WARN(this->get_logger(),"PID: Speed command has timed out. Last speed "
               "command is %lf seconds old. Timeout = %lf",
               age_comm,
	       command_timeout_);
    }

    return (odom_timed_out || comm_timed_out);
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


  void HandleSpeedCommand(const mcm::Float32Stamped::SharedPtr msg) /// TODO I had to change this from &msg to msg to compile - will it break?
  {
    last_speed_command_ = *msg;
    last_speed_command_.value = std::abs(msg->value);
    pid_.LoadCommanded(std::abs(msg->value));
  }

  GasBrakePid(rclcpp::NodeOptions options):
    Node("gas_brake_pid", options),
    odom_timeout_(0.0),
    command_timeout_(0.0),
    max_throttle_rate_(0.0),
    max_brake_rate_(0.0),
    parameters_initialized_(false),
    is_stopped_(false),
    last_throttle_command_(0.0),
    last_brake_command_(0.0),
    update_rate_hz_(25.0),
    parameters_updated_(false),
    control_map_config_(this->get_logger().get_child("control_map_config")),
    pid_config_(this->get_logger().get_child("pid_config")),
    pid_(this->get_logger().get_child("pid"),this->now())
  {
    last_odom_.header.stamp = this->now();
    last_speed_command_.header.stamp = this->now();
    // declare all parameters
    this->declare_parameter("initialization_delay_s", rclcpp::ParameterValue(1.0f));

    this->declare_parameter("kp", rclcpp::ParameterValue(0.0));
    this->declare_parameter("ki", rclcpp::ParameterValue(0.0));
    this->declare_parameter("kd", rclcpp::ParameterValue(0.0));
    this->declare_parameter("kp_stop", rclcpp::ParameterValue(0.0));
    this->declare_parameter("ki_stop", rclcpp::ParameterValue(0.0));
    this->declare_parameter("kd_stop", rclcpp::ParameterValue(0.0));
    this->declare_parameter("min_integral", rclcpp::ParameterValue(0.0));
    this->declare_parameter("max_integral", rclcpp::ParameterValue(0.0));
    this->declare_parameter("min_derivative", rclcpp::ParameterValue(0.0));
    this->declare_parameter("max_derivative", rclcpp::ParameterValue(0.0));
    this->declare_parameter("feedforward_vs_speed_ratio", rclcpp::ParameterValue(0.0));
    this->declare_parameter("measured_filter_cut_off_hz", rclcpp::ParameterValue(0.0));
    this->declare_parameter("derr_filter_cut_off_hz", rclcpp::ParameterValue(0.0));
    this->declare_parameter("verbose", rclcpp::ParameterValue(false));
    this->declare_parameter("odom_timeout", rclcpp::ParameterValue(0.5));
    this->declare_parameter("command_timeout", rclcpp::ParameterValue(0.5));
    this->declare_parameter("stop_velocity_threshold", rclcpp::ParameterValue(0.1));
    this->declare_parameter("max_throttle_rate", rclcpp::ParameterValue(0.0));
    this->declare_parameter("max_brake_rate", rclcpp::ParameterValue(0.0));
    this->declare_parameter("update_rate_hz", rclcpp::ParameterValue(25.0));
    this->declare_parameter("min_brake", rclcpp::ParameterValue(0.0));
    this->declare_parameter("max_brake", rclcpp::ParameterValue(1.0));
    this->declare_parameter("brake_stop", rclcpp::ParameterValue(0.6));
    this->declare_parameter("min_throttle", rclcpp::ParameterValue(0.0));
    this->declare_parameter("max_throttle", rclcpp::ParameterValue(1.0));
    this->declare_parameter("min_stop_brake", rclcpp::ParameterValue(0.5));

    // set up the parameter callback function
    //this->register_param_change_callback(std::bind(&GasBrakePid::ReconfigCb,this,std::placeholders::_1));
    this->set_on_parameters_set_callback(std::bind(&GasBrakePid::ReconfigCb,this,std::placeholders::_1));

    // wrap ROS1 nodelet initializer
    this->onInit();
  }
};  // class GasBrakePid

/*
/// I don't know what this did and couldn't find any uses of it .... ???
boost::shared_ptr<nodelet::Nodelet> createGasBrakePid()
{
  return boost::make_shared<GasBrakePid>();
}
*/
}  // namespace speed_controller

// Register nodelet plugin
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(speed_controller::GasBrakePid)
