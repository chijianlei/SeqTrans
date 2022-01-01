
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <diagnostic_updater/diagnostic_updater.h>

//util
#include <swri_nav_util/linear_interpolant.h>

//msgs
#include <nav_msgs/Odometry.h>
#include <marti_common_msgs/Float32Stamped.h>
//dynamic reconfig
#include <dynamic_reconfigure/server.h>
#include <speed_controller/GainSchedulePidConfig.h>

namespace du = diagnostic_updater;
namespace nm = nav_msgs;
namespace mcm = marti_common_msgs;

namespace speed_controller
{

class GainSchedulePID : public nodelet::Nodelet
{
 private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  boost::shared_ptr<du::Updater> diagnostic_updater_;

  ros::WallTimer init_timer_;
  ros::Timer control_loop_timer_;

  ros::Subscriber odom_sub_;
  ros::Subscriber speed_command_sub_;

  ros::Publisher brake_command_pub_;
  ros::Publisher throttle_command_pub_;

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
  ros::Time last_command_time_;
  double integral_contribution_;

  // Reconfigure
  boost::recursive_mutex reconfig_mutex_;
  typedef dynamic_reconfigure::Server<speed_controller::GainSchedulePidConfig> ReconfigureServer;
  boost::shared_ptr<ReconfigureServer> reconfig_server_;

 public:
  GainSchedulePID() :
    stop_speed_threshold_(0.1),
    min_integral_contribution_(0.0),
    max_integral_contribution_(0.0),
    i_gain_(0.0),
    feedforward_m_(0.0),
    feedforward_b_(0.0),
    min_gasbrake_command_(0.0),
    max_gasbrake_command_(0.0),
    first_iteration_(true),
    last_command_time_(ros::Time::now()),
    integral_contribution_(0.0)
  {
  }

  ~GainSchedulePID()
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
                                      &GainSchedulePID::initialize,
                                      this,
                                      true);
  }

  void initialize(const ros::WallTimerEvent &)
  {
    //subscriber
    odom_sub_ = nh_.subscribe(
      "odom", 1,
      &GainSchedulePID::HandleOdometry, this);

    speed_command_sub_ = nh_.subscribe(
      "speed_command", 1,
      &GainSchedulePID::HandleSpeedCommand, this);

    //publisher
    throttle_command_pub_ = nh_.advertise<mcm::Float32Stamped>(
      "throttle_command", 1, false);

    brake_command_pub_ = nh_.advertise<mcm::Float32Stamped>(
      "brake_command", 1, false);

    pnh_.param("odom_timeout", odom_timeout_, 0.5);
    pnh_.param("command_timeout", command_timeout_, 0.5);

    //parameters
    pnh_.param("stop_speed_threshold", stop_speed_threshold_, 0.0);
    pnh_.param("feedforward_m", feedforward_m_, 0.0);
    pnh_.param("feedforward_b", feedforward_b_, 0.0);

    // Parse gain scheduling
    speed_to_gain_p_.loadFromParameterServer(pnh_, "speed_to_gain_p");
    if (!speed_to_gain_p_.valid())
    {
      ROS_ERROR("Failed to load speed_to_gain_p from parameter server.");
      std::vector<double> zero = {0.0};
      speed_to_gain_p_.set(zero, zero);
    }
    ROS_INFO("speed_to_gain_p_ = %s", speed_to_gain_p_.toString().c_str());

    pnh_.param("min_integral_contribution", min_integral_contribution_, -10.0);
    pnh_.param("max_integral_contribution", max_integral_contribution_, 10.0);
    pnh_.param("integral_gain", i_gain_, 0.0);
    pnh_.param("min_gasbrake_command", min_gasbrake_command_, -100.0);
    pnh_.param("max_gasbrake_command", max_gasbrake_command_, 100.0);

    //dynamic reconfig
    reconfig_server_.reset(new ReconfigureServer(reconfig_mutex_,
        pnh_));
    reconfig_server_->setCallback(
        boost::bind(&GainSchedulePID::dynamicReconfigCallback,
            this, _1, _2));

    updateDynamicReconfig();

    //setup control loop timer
    double update_rate_hz;
    pnh_.param("update_rate_hz", update_rate_hz, 25.0);

    control_loop_timer_ = nh_.createTimer(ros::Duration(1.0 / update_rate_hz),
                                &GainSchedulePID::ControlLoop,
                                this);

    return;
  }

  void dynamicReconfigCallback(
      speed_controller::GainSchedulePidConfig& config,  // NOLINT: Non-const ref required.
      uint32_t level)
  {
    if (level == ~static_cast<uint32_t>(0))
      return;

    stop_speed_threshold_ = config.stop_speed_threshold;
    feedforward_m_ = config.feedforward_m;
    feedforward_b_ = config.feedforward_b;
    min_integral_contribution_ = config.min_integral_contribution;
    max_integral_contribution_ = config.max_integral_contribution;
    i_gain_ = config.integral_gain;
    min_gasbrake_command_ = config.min_gasbrake_command;
    max_gasbrake_command_ = config.max_gasbrake_command;

    if (!speed_to_gain_p_.fromString(config.speed_to_gain))
    {
      ROS_ERROR("Failed to set speed_to_gain to '%s'", config.speed_to_gain.c_str());
    }
    ROS_INFO("speed_to_gain_p_ = %s", speed_to_gain_p_.toString().c_str());
    speed_to_gain_p_.toString(config.speed_to_gain);

  }

  void updateDynamicReconfig()
  {
    boost::recursive_mutex::scoped_lock lock(reconfig_mutex_);
    speed_controller::GainSchedulePidConfig config;

    config.stop_speed_threshold = stop_speed_threshold_;
    config.feedforward_m = feedforward_m_;
    config.feedforward_b = feedforward_b_;
    config.min_integral_contribution = min_integral_contribution_;
    config.max_integral_contribution = max_integral_contribution_;
    config.integral_gain = i_gain_;
    config.min_gasbrake_command = min_gasbrake_command_;
    config.max_gasbrake_command = max_gasbrake_command_;
    speed_to_gain_p_.toString(config.speed_to_gain);

    reconfig_server_->updateConfig(config);
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
  void ControlLoop(const ros::TimerEvent &)
  {
    if (HasTimedOut())
    {
      return;
    }
    ros::Time now = ros::Time::now();
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
      ROS_DEBUG("gasbrake %f, feedforward %f, proportional %f, integral %f",
                gasbrake_percent, feedforward, proportional, integral);
      gasbrake_percent = ClampGasBrake(gasbrake_percent);

      first_iteration_ = false;
      last_command_time_ = now;
    }

    //publish
    mcm::Float32StampedPtr throttle_msg = boost::make_shared<mcm::Float32Stamped>();
    throttle_msg->header.stamp = now;
    throttle_msg->value = std::max(gasbrake_percent, 0.0)/100.0;
    throttle_command_pub_.publish(throttle_msg);

    mcm::Float32StampedPtr brake_msg = boost::make_shared<mcm::Float32Stamped>();
    brake_msg->header.stamp = now;
    brake_msg->value = std::max(-gasbrake_percent, 0.0)/100.0;
    brake_command_pub_.publish(brake_msg);
  }

 protected:
  double ProportionalTerm(double current_speed, double speed_error)
  {
    double scheduled_gain = speed_to_gain_p_.eval(current_speed);
    ROS_DEBUG("cur speed %f, speed error %f, scheduled gain %f, p term %f",
             current_speed, speed_error, scheduled_gain, scheduled_gain * speed_error);
    return scheduled_gain * speed_error;
  }

  double IntegralTerm(const ros::Time &now,
                    double speed_error)
  {
    if (first_iteration_) {
      integral_contribution_ = 0;
    } else {
      double dt = (now - last_command_time_).toSec();
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
    ros::Time now = ros::Time::now();

    double age_meas = (now - last_odom_.header.stamp).toSec();
    bool odom_timed_out = (age_meas > odom_timeout_);

    if (odom_timed_out)
    {
      ROS_WARN_THROTTLE(1.0, "PID: Odometry timed out. Last odom message is %lf "
         "seconds old. Timeout = %lf",
               age_meas,
         odom_timeout_);
    }

    double age_comm = (now - last_speed_command_.header.stamp).toSec();
    bool comm_timed_out = (age_comm > command_timeout_);
    if (comm_timed_out)
    {
      ROS_WARN_THROTTLE(1.0, "PID: Speed command has timed out. Last speed "
               "command is %lf seconds old. Timeout = %lf",
               age_comm,
         command_timeout_);
    }

    return (odom_timed_out || comm_timed_out);
  }

  void HandleOdometry(const nm::OdometryConstPtr &msg)
  {
    last_odom_ = *msg;
  }

  void HandleSpeedCommand(const mcm::Float32StampedConstPtr &msg)
  {
    last_speed_command_ = *msg;
    last_speed_command_.value = std::abs(msg->value);
  }
};
}  // namespace speed_controller

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(
    speed_controller::GainSchedulePID,
    nodelet::Nodelet)
