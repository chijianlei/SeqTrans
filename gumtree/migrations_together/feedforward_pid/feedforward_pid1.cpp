#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <diagnostic_updater/diagnostic_updater.h>

#include <nav_msgs/Odometry.h>
#include <marti_common_msgs/Float32Stamped.h>
#include <marti_common_msgs/BoolStamped.h>
#include <marti_dbw_msgs/TransmissionFeedback.h>

#include <speed_controller/longitudinal_dynamics.h>
#include <speed_controller/simple_pid.h>

#include <dynamic_reconfigure/server.h>
#include <speed_controller/FeedforwardPidConfig.h>

// Alias type for easier access to DiagnosticStatus enumerations.
typedef diagnostic_msgs::DiagnosticStatus DS;

namespace du = diagnostic_updater;
namespace nm = nav_msgs;
namespace mcm = marti_common_msgs;
namespace mdm = marti_dbw_msgs;

namespace speed_controller
{

class FeedforwardPid : public nodelet::Nodelet
{
 private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  boost::shared_ptr<du::Updater> diagnostic_updater_;

  ros::WallTimer init_timer_;
  ros::Timer control_loop_timer_;

  ros::Subscriber odom_sub_;
  ros::Subscriber speed_command_sub_;
  ros::Subscriber accel_command_sub_;
  ros::Subscriber transmission_sense_sub_;

  ros::Publisher brake_command_pub_;
  ros::Publisher throttle_command_pub_;
  ros::Publisher pid_measured_filtered_pub_; //DEBUGGING

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

  //persist between updates
  bool is_stopped_;

  dynamic_reconfigure::Server<speed_controller::FeedforwardPidConfig>* reconfigure_server_;
  speed_controller::FeedforwardPidConfig reconfig_;

 public:
  FeedforwardPid():
    odom_timeout_(0.0),
    command_timeout_(0.0),
    stop_velocity_thresh_(0.01),
    enable_feedforward_(true),
    parameters_initialized_(false),
    is_stopped_(false)
  {

  }

  void onInit()
  {
    nh_ = getNodeHandle();
    pnh_ = getPrivateNodeHandle();

    // Create a one-shot timer to initialize everything after a brief
    // pause so that ROS has time to connect to rosout so that we
    // don't drop errors/info during initialization.
    double initialization_delay = 1.0;
    pnh_.param("initialization_delay_s", initialization_delay, 1.0);
    init_timer_ = nh_.createWallTimer(ros::WallDuration(initialization_delay),
                                      &FeedforwardPid::initialize,
                                      this,
                                      true);
  }

  void initialize(const ros::WallTimerEvent &)
  {
    //Initialize subscribers
    odom_sub_ = nh_.subscribe("odom", 1,
      &FeedforwardPid::HandleOdometry, this);

    speed_command_sub_ = nh_.subscribe("speed_command", 1,
      &FeedforwardPid::HandleSpeedCommand, this);

    accel_command_sub_ = nh_.subscribe("acceleration_command", 1,
      &FeedforwardPid::HandleAccelerationCommand, this);

    transmission_sense_sub_ = nh_.subscribe("transmission_sense", 1,
      &FeedforwardPid::HandleTransmissionSense, this);

    //Initialize publishers
    throttle_command_pub_ = nh_.advertise<mcm::Float32Stamped>(
      "throttle_command", 1, false);

    brake_command_pub_ = nh_.advertise<mcm::Float32Stamped>(
      "brake_command", 1, false);

    pid_measured_filtered_pub_ = nh_.advertise<mcm::Float32Stamped>(
      "pid_measured_filtered", 1, false); //DEBUGGING

    //parameters
    pnh_.param("odom_timeout", odom_timeout_, 0.1);
    pnh_.param("command_timeout", command_timeout_, 0.1);
    pnh_.param("stop_velocity_threshold", stop_velocity_thresh_, 0.01);
    pnh_.param("enable_feedforward", enable_feedforward_, true);

    // Initialize LongitudinalDynamicsModel
    mdl_.getParams(pnh_);

    //Initialize PID
    double kp, ki, min_i, max_i, meas_hz;
    pnh_.param("kp", kp, 0.0);
    pnh_.param("ki", ki, 0.0);
    pnh_.param("min_integral", min_i, 0.0);
    pnh_.param("max_integral", max_i, 0.0);
    pnh_.param("measured_filter_cut_off_hz", meas_hz, 0.0);

    pid_config_.SetGains(kp, ki, 0.0);
    pid_config_.SetLimits(min_i, max_i, 0.0, 0.0);
    pid_config_.SetFilterCutOffHz(meas_hz, 0.0);
    pid_config_.PrintConfig();
    pid_.SetConfig(pid_config_);

    pnh_.param("stop_brake", stop_brake_, 0.5);

    //initialize diagnostic updater
    diagnostic_updater_ = boost::make_shared<du::Updater>(nh_, pnh_, getName());
    diagnostic_updater_->setHardwareID("none");
    diagnostic_updater_->add(
      "Feedforward PID", this,
      &FeedforwardPid::updateDiagnostics);

    //Initialize dynamic reconfig
    LoadParametersToReconfig();
    reconfigure_server_ =
      new dynamic_reconfigure::Server<speed_controller::FeedforwardPidConfig>(pnh_);
    reconfigure_server_->updateConfig(reconfig_);
    reconfigure_server_->setCallback(
      boost::bind(&FeedforwardPid::ReconfigCb, this, _1, _2));

    //Initialize update timer
    double update_rate_hz;
    pnh_.param("update_rate_hz", update_rate_hz, 20.0);

    control_loop_timer_ = nh_.createTimer(ros::Duration(1.0 / update_rate_hz),
                                &FeedforwardPid::ControlLoop,
                                this);
  }

  void ReconfigCb(
      speed_controller::FeedforwardPidConfig& config,  // NOLINT: Non-const ref required.
      uint32_t level)
  {
    if (!parameters_initialized_)
    {
      return;
    }

    reconfig_ = config;

    enable_feedforward_ = reconfig_.enable_feedforward;

    //Update LongitudinalDynamicsModel
    mdl_.delay = reconfig_.delay;
    mdl_.throttle_deadband = reconfig_.throttle_deadband;
    mdl_.throttle_gain = reconfig_.throttle_gain;
    mdl_.idle_power = reconfig_.idle_power;
    mdl_.speed_offset = reconfig_.speed_offset;
    mdl_.peak_force = reconfig_.peak_force;
    mdl_.brake_deadband = reconfig_.brake_deadband;
    mdl_.brake_gain = reconfig_.brake_gain;
    mdl_.brake_gain2 = reconfig_.brake_gain2;
    mdl_.resistance_coeff_1 = reconfig_.resistance_coeff_1;
    mdl_.resistance_coeff_v = reconfig_.resistance_coeff_v;
    mdl_.resistance_coeff_v2 = reconfig_.resistance_coeff_v2;
    mdl_.grav_gain = reconfig_.grav_gain;

    //Update PID
    pid_config_.SetGains(
      reconfig_.kp,
      reconfig_.ki,
      0.0);

    pid_config_.SetLimits(
      reconfig_.min_integral,
      reconfig_.max_integral,
      0.0, 0.0);

    pid_config_.SetFilterCutOffHz(
      reconfig_.measured_filter_cut_off_hz, 0.0);

    pid_config_.PrintConfig();

    pid_.SetConfig(pid_config_);

    stop_brake_ = reconfig_.stop_brake;

    reconfigure_server_->updateConfig(reconfig_);
  }

  void LoadParametersToReconfig()
  {
    reconfig_.enable_feedforward = enable_feedforward_;

    // Load LongitudinalDynamicsModel parameters
    reconfig_.delay = mdl_.delay;
    reconfig_.throttle_deadband = mdl_.throttle_deadband;
    reconfig_.throttle_gain = mdl_.throttle_gain;
    reconfig_.idle_power = mdl_.idle_power;
    reconfig_.speed_offset = mdl_.speed_offset;
    reconfig_.peak_force = mdl_.peak_force;
    reconfig_.brake_deadband = mdl_.brake_deadband;
    reconfig_.brake_gain = mdl_.brake_gain;
    reconfig_.brake_gain2 = mdl_.brake_gain2;
    reconfig_.resistance_coeff_1 = mdl_.resistance_coeff_1;
    reconfig_.resistance_coeff_v = mdl_.resistance_coeff_v;
    reconfig_.resistance_coeff_v2 = mdl_.resistance_coeff_v2;
    reconfig_.grav_gain = mdl_.grav_gain;

    // Load PID parameters
    reconfig_.kp = pid_config_.GetKp();
    reconfig_.ki = pid_config_.GetKi();
    reconfig_.min_integral = pid_config_.GetIntegralMin();
    reconfig_.max_integral = pid_config_.GetIntegralMax();
    reconfig_.measured_filter_cut_off_hz = pid_config_.GetMeasuredFilterCutOffHz();

    reconfig_.stop_brake = stop_brake_;

    parameters_initialized_ = true;
  }

  void updateDiagnostics(du::DiagnosticStatusWrapper& status)
  {
    status.summary(DS::OK, "No errors reported.");
  }

  void ControlLoop(const ros::TimerEvent &ev)
  {
    if (HasTimedOut())
    {
      ROS_WARN_THROTTLE(1.0, "Subscriber has timed out.");
      return;
    }

    ros::Time now = ros::Time::now();
//    ros::Time now = ev.current_expected; //use this for regular time steps

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
          ROS_WARN("Failed call to calcThrottleBrake(acceleration = %lf, velocity = %lf, pitch = %lf, in_reverse = %d)",
              acceleration, velocity, pitch, in_reverse); //DEBUGGING
          gasbrake_ff = 0.0; //should already be zero
        }
      }

      //get raw throttle & brake commands from pid
      double gasbrake_pid = pid_.Update(now);
      double gasbrake = gasbrake_ff + gasbrake_pid;

      ROS_DEBUG("cmd speed = %lf, acceleration = %lf, meas speed = %lf, gasbrake = %lf (feedforward = %lf, pid = %lf)",
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
    throttle_command_pub_.publish(makeFloat32StampedPtr(now, throttle_cmd));
    brake_command_pub_.publish(makeFloat32StampedPtr(now, brake_cmd));
    pid_measured_filtered_pub_.publish(makeFloat32StampedPtr(now, pid_.GetMeasuredFiltered())); //DEBUGGING

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

  void HandleOdometry(const nm::OdometryConstPtr &msg)
  {
    last_odom_ = *msg;
    double speed = std::abs(msg->twist.twist.linear.x);
    //TODO, instead of taking absolute value, would it be better to
    //subscribe to transmission feedback and negate if in reverse.
    //that way speed controller can tell if moving in wrong direction.
    pid_.LoadMeasured(speed);
  }

  void HandleSpeedCommand(const mcm::Float32StampedConstPtr &msg)
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
  void HandleAccelerationCommand(const mcm::Float32StampedConstPtr &msg)
  {
    last_accel_command_ = *msg;
  }

  void HandleTransmissionSense(const mdm::TransmissionFeedbackConstPtr &msg)
  {
    last_transmission_sense_ = *msg;
  }

};  // class FeedforwardPid

boost::shared_ptr<nodelet::Nodelet> createFeedForwardPid()
{
  return boost::make_shared<FeedforwardPid>();
}
}  // namespace

// Register nodelet plugin
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(
    speed_controller::FeedforwardPid,
    nodelet::Nodelet)
