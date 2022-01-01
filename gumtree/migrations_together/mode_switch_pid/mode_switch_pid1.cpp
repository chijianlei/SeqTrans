#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <diagnostic_updater/diagnostic_updater.h>

//util
#include <swri_nav_util/linear_interpolant.h>

//msgs
#include <nav_msgs/Odometry.h>
#include <marti_common_msgs/Float32Stamped.h>
//dynamic reconfig not implemented

namespace du = diagnostic_updater;
namespace nm = nav_msgs;
namespace mcm = marti_common_msgs;

namespace speed_controller
{

class ModeSwitchPid : public nodelet::Nodelet
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
  double brake_error_threshold_;
  int brake_saturated_threshold_;
  double brake_kp_;
  double brake_ki_;
  double brake_integral_;
  double max_brake_integral_;
  double min_brake_integral_;
  ros::Time brake_last_time_;
  int brake_saturated_count_;

  double throttle_error_threshold_;
  int throttle_saturated_threshold_;
  double throttle_kp_;
  double throttle_ki_;
  double throttle_integral_;
  double max_throttle_integral_;
  double min_throttle_integral_;
  ros::Time throttle_last_time_;
  int throttle_saturated_count_;

  enum SpeedControllerState {
    STATE_STOP,
    STATE_APPLY_THROTTLE,
    STATE_APPLY_BRAKE
  };

  SpeedControllerState state_;
  SpeedControllerState last_state_;

 public:
  ModeSwitchPid()
    :
    brake_error_threshold_(0.0),
    brake_saturated_threshold_(0.0),
    brake_kp_(0.0),
    brake_ki_(0.0),
    brake_integral_(0.0),
    max_brake_integral_(0.0),
    min_brake_integral_(0.0),
    brake_last_time_(ros::Time::now()),
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
  }

  ~ModeSwitchPid()
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
                                      &ModeSwitchPid::initialize,
                                      this,
                                      true);
  }

  void initialize(const ros::WallTimerEvent &)
  {
    //subscriber
    odom_sub_ = nh_.subscribe(
      "odom", 1,
      &ModeSwitchPid::HandleOdometry, this);

    speed_command_sub_ = nh_.subscribe(
      "speed_command", 1,
      &ModeSwitchPid::HandleSpeedCommand, this);

    //publisher
    throttle_command_pub_ = nh_.advertise<mcm::Float32Stamped>(
      "throttle_command", 1, false);

    brake_command_pub_ = nh_.advertise<mcm::Float32Stamped>(
      "brake_command", 1, false);

    pnh_.param("odom_timeout", odom_timeout_, 0.5);
    pnh_.param("command_timeout", command_timeout_, 0.5);

    //parameters
    pnh_.param("brake_error_threshold", brake_error_threshold_, 0.0);
    pnh_.param("brake_saturated_threshold", brake_saturated_threshold_, 0);
    pnh_.param("brake_kp", brake_kp_, 0.0);
    pnh_.param("brake_ki", brake_ki_, 0.0);
    pnh_.param("max_brake_integral", max_brake_integral_, 0.0);
    pnh_.param("min_brake_integral", min_brake_integral_, 0.0);
    pnh_.param("throttle_error_threshold", throttle_error_threshold_, 0.0);
    pnh_.param("throttle_saturated_threshold", throttle_saturated_threshold_, 0);
    pnh_.param("throttle_kp", throttle_kp_, 0.0);
    pnh_.param("throttle_ki", throttle_ki_, 0.0);
    pnh_.param("max_throttle_integral", max_throttle_integral_, 0.0);
    pnh_.param("min_throttle_integral", min_throttle_integral_, 0.0);

    //setup control loop timer
    double update_rate_hz;
    pnh_.param("update_rate_hz", update_rate_hz, 25.0);

    control_loop_timer_ = nh_.createTimer(ros::Duration(1.0 / update_rate_hz),
                                &ModeSwitchPid::ControlLoop,
                                this);

    return;
  }

  void Shutdown()
  {
  }

  void Disable()
  {
    state_ = STATE_STOP;
  }

  void ControlLoop(const ros::TimerEvent &)
  {
    if (HasTimedOut())
    {
      return;
    }
    ros::Time now = ros::Time::now();
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
  double RunStop(double desired)
  {
    if (desired != 0.0) {
      state_ = STATE_APPLY_BRAKE;
    }

    return -50.0;
  }

  double RunThrottle(const ros::Time &now, double desired, double measured)
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
  
    double dt = (now - throttle_last_time_).toSec();
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

  double RunBrake(const ros::Time &now, double desired, double measured)
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

    double dt = (now - brake_last_time_).toSec();
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
    speed_controller::ModeSwitchPid,
    nodelet::Nodelet)
