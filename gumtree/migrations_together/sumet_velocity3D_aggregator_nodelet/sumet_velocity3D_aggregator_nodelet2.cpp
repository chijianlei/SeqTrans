// -----------------------------------------------------------------------------
// SECURITY CLASSIFICATION: UNCLASSIFIED
// -----------------------------------------------------------------------------
//
// Copyright (C) 2012 Southwest Research Institute
//
// Notwithstanding any copyright notice, U.S. Government rights in this work are
// defined by 252.227-7013 (f)(2) and 252.227-7014 (f)(2) as detailed below. Use
// of this work other than as specifically authorized by the U.S. Government may
// violate any copyrights that exist in this work.
//
// UNLIMITED RIGHTS
// DFARS Clause reference: 252.227-7013 (a)(15) and 252.227-7014 (a)(15)
//
// Unlimited Rights. The Government has the right to use, modify, reproduce,
// perform, display, release or disclose this (technical data or computer
// software) in whole or in part, in any manner, and for any purpose whatsoever,
// and to have or authorize others to do so.
//
// Contract No.  NOO178-11-C-1005
// Contractor    Southwest Research Institute® (SwRI®)
// Address       6220 Culebra Road, San Antonio, Texas 78228-0510
//
// Distribution Statement D. Distribution authorized to the Department of
// Defense and U.S. DoD Contractors only in support of US DoD efforts. Other
// requests shall be referred to [PEO].
//
// Warning: - This document contains data whose export is restricted by the Arms
// Export Control Act (Title 22, U.S.C., Sec 2751, et seq.) as amended, or the
// Export Administration Act (Title 50, U.S.C., App 2401 et seq.) as amended.
// Violations of these export laws are subject to severe criminal and civil
// penalties. Disseminate in accordance with provisions of DoD Directive 5230.25
//

/**
 * \file
 *
 * The velocity aggregator collects all
 * velocity-type signals, averages all of the relevant data component-wise
 * on a fixed frequency and then publishes a Twist message that corresponds
 * to the overall best estimate of system velocity at the time (along with
 * associated covariances).
 *
 *   - <b>Subscribed Topics</b>
 *      - \e imu/data <tt>[sensor_msgs::Imu]</tt> - This is the message
 *      expected from the Microstrain IMU node (or from the
 *      microstrain_filter_node).
 *      - \e gps <tt>[marti_gps_common::GPSFix]</tt> - This is the message
 *      expected from the GPS receiver.
 *      - \e kvh_gyro <tt>[sensor_msgs::Imu]</tt> - Message from the gyroscope
 *      driver. This topic is deprecated. Use \e yaw_rate.
 *      - \e yaw_rate <tt>[marti_sensor_msgs::Gyro]</tt> - Message from the
 *      gyroscope driver. Use this *or* \e kvh_gyro.
 *      - \e speed <tt>[marti_sensor_msgs::Velocity]</tt> - Vehicle velocity
 *       from the wheel speed encoders. 
 *
 *   - <b>Published Topics</b>
 *      - \e VehicleVelocity6D <tt>[geometry_msgs::TwistWithCovarianceStamped]</tt> -
 *      The estimate of vehicle linear and angular velocity along with the
 *      associated covariances.
 *
 *   - \b Parameters
 *      - \e initial_latitude <tt>[double]</tt> - An initial
 *      latitude to use for earth-rate compensation of the gyro.  If GPS is
 *      available, an updated latitude will be used from the GPSFix messages.
 *      [29.44]
 *      - \e verbose <tt>[bool]</tt> - If true, more information is
 *      written to the console during run time. [false]
 *      - \e timeout_warning <tt>[double]</tt> - If no IMU messages
 *      have been received in this amount of time, a warning will be printed to
 *      the screen. [1.0]
 *      - \e track_width <tt>[double]</tt> - The vehicle track width, i.e. the
 *      distance between the center of the rear tires.  [1.829 -- HMMWV Track];
 *      - \e wheel_slip_compensation_ratio <tt>[double]</tt> - The amount of
 *      slippage to assign to the outside vs. inside wheel.  1.0 means that
 *      only the outside wheel slips, while -1.0 means that only the inside
 *      wheel slips, and 0.0 does no compensation.  [0.0]
 */

// ROS Libraries
#include <rclcpp/rclcpp.hpp>
#include <swri_roscpp/node.h>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <diagnostic_updater/diagnostic_updater.h>

// MARTI Libraries
#include <gps_common_msgs/msg/gps_fix.hpp>
#include <marti_sensor_msgs/msg/gyro.hpp>
#include <marti_sensor_msgs/msg/velocity.hpp>

// SUMET Libraries
#include <sumet_util/math_util.h>

#include <sumet_state_estimator/VelocityStreamList3D.h>
#include <sumet_state_estimator/sumet_state_estimator_types.h>
#include <sumet_state_estimator/BiasCalcClass.h>

#include <marti_dbw_msgs/msg/transmission_feedback.hpp>

// Boost libraries
//#include <std/shared_ptr.hpp>

// SwRI Libraries
#include <swri_roscpp/parameters.h>
#include <swri_roscpp/publisher.h>
#include <swri_roscpp/subscriber.h>
#include <swri_roscpp/time.h>


namespace sumet_state_estimator
{
typedef std::vector<bool> bvec;

class TwistAggregator : public swri::Node
{
 public:
  TwistAggregator()
    :
    Microstrain_("microstrain"),
    Novatel_("novatel"),
    DSP3000_("kvh_gyro"),
    CAN_WS_Velocity_("can_ws_velocity"),
    CAN_WS_Angular_("can_ws_angular"),
    CAN_Angular_("can_angular"),
    DT_(0.01),
    cur_latitude_(29.44),
    last_odom_time_(0, 0, RCL_ROS_TIME),
    //last_published_twist_(new geometry_msgs::msg::TwistWithCovarianceStamped()),
    Node("twist_aggregator")
  {
    verbose_ = false;
    timeout_warning_ = 1.0;
    gyro_bias_estimation_time_ = 15.0;
    gyro_error_threshold_ = -1.0;
    sampling_rate_ = 100.0;
    rear_passenger_ws_cal_factor_ = 1.0;
    rear_driver_ws_cal_factor_ = 1.0;
    long_vel_timeout_ = 0.75;
    yaw_rate_timeout_ = 0.5;
    track_width_ = 1.30;
    wheel_slip_compensation_ratio_ = 0.0;
    local_earth_rate_ = (sumet_util::_earth_rate *
                         sumet_util::MathUtil::sind(cur_latitude_));
    curSpeed_ = 0;
    in_reverse_ = false;

    last_published_twist_.header.stamp = rclcpp::Time(0, 0);
  }

 private:
  // Subscriptions
  swri::Subscriber Microstrain_Subscriber_;
  swri::Subscriber Novatel_Subscriber_;
  swri::Subscriber Gyro_Subscriber_;
  swri::Subscriber Wheel_Speed_Subscriber_;
  swri::Subscriber Visual_Odom_Msg_Subscriber_;
  swri::Subscriber transmission_sense_sub_;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr velocity_publisher_;

  // Variables for topic names
  const std::string Microstrain_;
  const std::string Novatel_;
  const std::string DSP3000_;
  const std::string CAN_WS_Velocity_;
  const std::string CAN_WS_Angular_;
  const std::string CAN_Angular_;

  // Timer to drive the output
  rclcpp::TimerBase::SharedPtr output_timer_;

  // Primary list of recently acquired velocity (linear and angular) data
  sumet_state_estimator::VelocityStreamList3D VSL_;

  //
  geometry_msgs::msg::TwistWithCovarianceStamped last_twist_;

  geometry_msgs::msg::Vector3Stamped last_microstrain_vel_;

  double DT_;
  double sampling_rate_;
  double cur_latitude_;
  double local_earth_rate_;

  // Time to wait before warning of a lost velocity stream
  double timeout_warning_;

  // Safety timeouts
  bool require_wheel_encoders_;
  rclcpp::Time last_odom_time_;
  static constexpr double ODOM_TIMEOUT = 0.1;  // Seconds

  double curSpeed_;
  double gyro_bias_estimation_time_;
  double gyro_error_threshold_;
  double rear_driver_ws_cal_factor_;
  double rear_passenger_ws_cal_factor_;

  // Diagnostics
  std::shared_ptr<diagnostic_updater::Updater> diagnostic_updater_;
  rclcpp::TimerBase::SharedPtr diagnostic_timer_;

  rclcpp::Time last_longitudinal_vel_update_ = swri::TIME_MIN;
  rclcpp::Time last_yaw_rate_update_ = swri::TIME_MIN;

  // Diagnostic Parameters
  double long_vel_timeout_;
  double yaw_rate_timeout_;
  double track_width_;
  double wheel_slip_compensation_ratio_;
  bool ignore_visual_odometry_diagnostic_;

  bool verbose_;

  sumet_state_estimator::BiasCalcClass gyro_bias_;

  geometry_msgs::msg::TwistWithCovarianceStamped last_published_twist_;

  bool in_reverse_;
  
  void onInit()
  {
    diagnostic_updater_ = std::make_shared<diagnostic_updater::Updater>(
        nh_, nh_->get_name());
    // Initialize the node
    init_node();

    // Publish topics
    publish_topics();

    // Subscribe to topics
    subscribe_to_topics();

    setup_output_timer();
  }

  /**
   * @brief Extracts angular velocity, acceleration and orientation data from
   * Microstrain and republishes the relevant bits.
   *
   * @param[in]   msg           Imu message from the Microstrain node
   */
  void Microstrain_cb(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    // Add the yaw rate to the appropriate velocity stream
    // First create a VelocityElem3D
    sumet_state_estimator::VelocityElem3D temp_elem;

    // Add the timestamp to the element
    temp_elem.set_timestamp(msg->header.stamp);

    // Extract the angular velocity and covariances from the message
    tf2::Vector3 w;
    tf2::Vector3 w_var;

    w.setValue(msg->angular_velocity.x,
               -msg->angular_velocity.y,
               -(msg->angular_velocity.z + local_earth_rate_));

    // Save this update so that we can use it in case we have a gap in updates
    last_microstrain_vel_.vector.x = w.x();
    last_microstrain_vel_.vector.y = w.y();
    last_microstrain_vel_.vector.z = w.z();
    last_microstrain_vel_.header.stamp = msg->header.stamp;

    w_var.setValue(msg->angular_velocity_covariance[0],
                   msg->angular_velocity_covariance[4],
                   msg->angular_velocity_covariance[8]);

    temp_elem.load_w(w,
                     w_var,
                     set_validity(true, true, true));

    VSL_.load_new_data(Microstrain_,
                       temp_elem);
  }



  /**
   * @brief       Extracts the speed from the GPS message
   *
   * @param[in]   msg           Imu message from the Novatel GPS node
   */
  void Novatel_cb(const gps_common_msgs::msg::GPSFix::SharedPtr msg)
  {
    // Add the yaw rate to the appropriate velocity stream
    // First create a VelocityElem3D
    sumet_state_estimator::VelocityElem3D temp_elem;

    // Add the timestamp to the element
    temp_elem.set_timestamp(msg->header.stamp);

    double speed = msg->speed;
    if (in_reverse_)
      speed = -speed;

    // Extract the angular velocity and covariances from the message
    tf2::Vector3 v;
    tf2::Vector3 v_var;
    v.setValue(speed,
               0.0,
               0.0);
    v_var.setValue(10.0,
                   1e20,
                   1e20);
    temp_elem.load_v(v,
                     v_var,
                     set_validity(true, false, false));
    VSL_.load_new_data(Novatel_,
                       temp_elem);

    curSpeed_ = msg->speed;

    // We'll also update the current latitude (this doesn't need to be very
    // precise, since it it is just used to compensate for the local earth rate
    // with the gyros

    cur_latitude_ = msg->latitude;
    local_earth_rate_ = sumet_util::_earth_rate *
        sumet_util::MathUtil::sind(cur_latitude_);
  }

  /**
   * @brief Process the angular velocity (yaw rate) from the fiber optic gyro
   *    using the new message type. Assumes that the FOG is pointed straight
   *    up for simplicity.
   * @param[in] msg IMU message from the KVH DSP Gyro node(let)
   */
  void gyro_cb(const marti_sensor_msgs::msg::Gyro::SharedPtr msg)
  {
    // Constant very low variance that Kris Kozak made up
    const double DSP3000_var = 0.0001;

    // Negate the yaw rate and subtract out the earth rate
    // (note that earth rate is measured as a negative value)
    double w_temp = msg->angular_rate - local_earth_rate_;

    // Added to address observed cases of faulty (very high) gyro
    // measurements. Note: The very high measurements were eventually
    // determined to be due to an undetected comms error.  That was
    // fixued, so it may be worth revisiting and removing this?
    if (gyro_error_threshold_ > 0 && std::abs(w_temp) > gyro_error_threshold_)
    {
      ROS_WARN("%s: Discarding gyro measurement above threshold %g > %g",
                   nh_->get_name(),
                   w_temp,
                   gyro_error_threshold_);
      return;
    }

    // If the vehicle is not moving, use this measuremnt to update the bias
    if (curSpeed_ == 0.0)
    {
      if (gyro_bias_.load_new_data(w_temp, nh_->now()))
      {
        double cur_bias = gyro_bias_.get_current_bias();
        double deg_per_hour = sumet_util::MathUtil::ToDegrees(cur_bias)*3600.0;
        ROS_INFO_THROTTLE(1.0, "%s: Initial gyro bias computed: %g rad/hour",
                              nh_->get_name(),
                              deg_per_hour);
      }
    }

    // Subtract out calculated bias
    w_temp -= gyro_bias_.get_current_bias();

    // Load the angular velocity and variance into Vector3s
    tf2::Vector3 w;
    w.setValue(0.0, 0.0, w_temp);
    tf2::Vector3 w_var;
    w_var.setValue(1e20, 1e20, DSP3000_var);

    // Load the Vector3s into a velocity element
    sumet_state_estimator::VelocityElem3D temp_elem;
    temp_elem.set_timestamp(msg->header.stamp);
    temp_elem.load_w(w,
                     w_var,
                     set_validity(false, false, true));

    // Load the velocity element into the velocity stream list
    VSL_.load_new_data(DSP3000_, temp_elem);
  }

  /**
  * @brief  Callback for wheel speed from the wheel encoders
  *
  * @param[in]   msg  Velocity message from the wheel encoder pipeline
  */
  void speed_cb(const marti_sensor_msgs::msg::Velocity::SharedPtr msg)
  {
    // Wheel speed measurements
    sumet_state_estimator::VelocityElem3D temp_elem;
    temp_elem.set_timestamp(msg->header.stamp);

    double v_wheel_speed = msg->velocity;
    double v_wheel_speed_var = msg->variance;
    if (v_wheel_speed_var == 0.0)  // Unknown variance
    {
      v_wheel_speed_var = 0.6;
    }
    // Wheel slip compensation
    /*if (std::fabs(swri::toSec(rclcpp::Time(last_published_twist_.header.stamp) - rclcpp::Time(msg->header.stamp))) < 0.1)  // NOLINT
    {
      if (v_wheel_speed > 0.25)
      {
        v_wheel_speed += wheel_slip_compensation_ratio_ *
            track_width_ *
            std::fabs(last_published_twist_.twist.twist.angular.z) / 2.0;
      }
    }*/

    tf2::Vector3 v(v_wheel_speed,
        0.0,
        0.0);
    tf2::Vector3 v_var(v_wheel_speed_var,
        sumet_util::_large_variance,
        sumet_util::_large_variance);
    temp_elem.load_v(v, v_var, set_validity(true, false, false));
    VSL_.load_new_data(CAN_WS_Velocity_, temp_elem);
    last_odom_time_ = msg->header.stamp;

    // Store the current speed for other processes that need it.
    curSpeed_ = v_wheel_speed;
  }

  void transmission_sense_cb(
    const marti_dbw_msgs::msg::TransmissionFeedback::SharedPtr msg)
  {
    in_reverse_ = msg->reverse;
  }

  /**
   * @brief   Checks whether all required sensors have updated recently enough
   * @return  True if the sensor data is current. False if it has timed out
   */
  bool CheckRequiredSensorTimeouts()
  {
    if (require_wheel_encoders_ &&
        ((nh_->now() - last_odom_time_).nanoseconds() > ODOM_TIMEOUT*1000000000) )
    {
      ROS_WARN_THROTTLE(1.0, "%s: Wheel speed odometry has timed out.",
                            nh_->get_name());
      return false;
    }
    return true;
  }

  /**
   * @brief This timer triggers the output of velocity measurements
   */
  void TimerCallback()
  {
    rclcpp::Time tmp_time = nh_->now();

    geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr temp_out(
      new geometry_msgs::msg::TwistWithCovarianceStamped());

    // May want to grab the time associated with the last message received
    // rather than "now"
    temp_out->header.stamp = tmp_time;

    // Compute the averages
    VSL_.get_averaged_twist(tmp_time,
                            temp_out->twist);

    check_and_fix_twist(*temp_out);

    if ( CheckRequiredSensorTimeouts() )
    {
      velocity_publisher_->publish(temp_out);
      last_published_twist_ = *temp_out;
    }
    else
    {
      ROS_ERROR_THROTTLE(1.0, "%s: Blocking output velocity because a "
                             "required sensor input is missing",
                             nh_->get_name());
    }
  }

  void check_and_fix_twist(geometry_msgs::msg::TwistWithCovarianceStamped& twist_in)
  {
    // Need to check to make sure that all the fields that need to be populated
    // are in fact populated

    double long_vel_cov;
    double yaw_rate_cov;
    double roll_rate_cov;
    double pitch_rate_cov;

    // Check longitudinal velocity
    long_vel_cov   = sumet_state_estimator::get_cov_by_idx_6x6(
                       twist_in.twist.covariance,
                       0,
                       0);

    roll_rate_cov  = sumet_state_estimator::get_cov_by_idx_6x6(
                       twist_in.twist.covariance,
                       3,
                       3);

    pitch_rate_cov = sumet_state_estimator::get_cov_by_idx_6x6(
                       twist_in.twist.covariance,
                       4,
                       4);

    yaw_rate_cov   = sumet_state_estimator::get_cov_by_idx_6x6(
                       twist_in.twist.covariance,
                       5,
                       5);

    // TODO(kkozak): implement timeout check (timeout_warning_)
    if (long_vel_cov > 1e2)
    {
      // Use last good twist value:
      twist_in.twist.twist.linear.x = last_twist_.twist.twist.linear.x;
      sumet_state_estimator::set_cov_by_idx_6x6(twist_in.twist.covariance,
                                                    0,
                                                    0,
                                                    0.5);

      if (verbose_)
      {
        ROS_ERROR_THROTTLE(
          1.0, 
          "%s: Longitudinal velocity out of date -- Using previous velocity."
          "  If you see this message frequently it may mean that the velocity "
          "update is unavailable or that there is a timing problem (velocity "
          "data may lagging, in which case the averaging window should be "
          "expanded)",
          nh_->get_name());
      }
    }
    else
    {
      last_longitudinal_vel_update_ = nh_->now();
    }

    if (yaw_rate_cov > 1e2)
    {
      // Use the last good twist value:
      twist_in.twist.twist.angular.z = last_twist_.twist.twist.angular.z;
      sumet_state_estimator::set_cov_by_idx_6x6(twist_in.twist.covariance,
                                                    5,
                                                    5,
                                                    0.001);

      if (verbose_)
      {
        ROS_ERROR_THROTTLE(
          1.0,
          "%s: Yaw rate out of date -- Using previous velocity."
          "  If you see this message frequently it may mean that the velocity "
          "update is unavailable or that there is a timing problem (velocity "
          "data may lagging, in which case the averaging window should be "
          "expanded)",
           nh_->get_name());
      }
    }
    else
    {
      last_yaw_rate_update_ = nh_->now();
    }

    if (roll_rate_cov > 1e6 || pitch_rate_cov > 1e6)
    {
      // TODO(evenator): Figure out if this code even executes in the case of
      // IMU death. If not, the aggregator is just happily republishing the
      // last pitch and roll, even if it's invalid due to age.
      ROS_ERROR_THROTTLE(10.0, "%s: Pitch and roll rates not valid",
                              nh_->get_name());

      // TODO(evenator): These should not assume constant pitch rate and
      // roll rate. 0 is a better assumption. Also, the variances might not be
      // well-chosen.
      twist_in.twist.twist.angular.x = last_microstrain_vel_.vector.x;
      twist_in.twist.twist.angular.y = last_microstrain_vel_.vector.y;
      sumet_state_estimator::set_cov_by_idx_6x6(twist_in.twist.covariance,
                                                    3,
                                                    3,
                                                    1.0);
      sumet_state_estimator::set_cov_by_idx_6x6(twist_in.twist.covariance,
                                                    4,
                                                    4,
                                                    1.0);
    }
    
    double delay = swri::toSec(rclcpp::Time(twist_in.header.stamp) -
                    rclcpp::Time(last_microstrain_vel_.header.stamp));
    if (delay > timeout_warning_)
    {
      ROS_ERROR_THROTTLE(10.0, "%s: IMU may be dead.  Last update %f seconds ago.",
                             nh_->get_name(),
                             delay);
    }

    last_twist_ = twist_in;
  }

  /**
   * @brief       Subscribes to relevant topics that have velocity-type
   *              measurements
   *
   */
  void subscribe_to_topics()
  {
    Microstrain_Subscriber_ = swri::Subscriber(this,
        "imu/data",
        100,
        &TwistAggregator::Microstrain_cb,
        this);

    Novatel_Subscriber_ = swri::Subscriber(this,
        "gps",
        100,
        &TwistAggregator::Novatel_cb,
        this);

    Gyro_Subscriber_ = swri::Subscriber(this,
        "yaw_rate",
        100,
        &TwistAggregator::gyro_cb,
        this);

    Wheel_Speed_Subscriber_ = swri::Subscriber(this,
        "speed",
        100,
        &TwistAggregator::speed_cb,
        this);

    transmission_sense_sub_ = swri::Subscriber(this,
      "/vehicle_interface/transmission_sense", 1,
      &TwistAggregator::transmission_sense_cb,
      this);
  }

  /**
   * @brief       Publishes all topics
   *
   */
  void publish_topics()
  {
    // Primary output of this node
    velocity_publisher_ =
      swri::advertise<geometry_msgs::msg::TwistWithCovarianceStamped> (this,
        "VehicleVelocity6D", 100);
  }


  void setup_velocity_stream_list()
  {
    // This was the old method for setting the window -- it will catch any
    // streams that aren't set below (0.05 - 0.1 seems to work for most
    // cases).
    VSL_.set_averaging_window(0.1);


    // Setup Linear Velocity Streams:
    VSL_.init_add_stream_def(Novatel_,
                             set_validity(true, false, false),
                             set_validity(false, false, false),
                             sumet_state_estimator::VelIgnore,
                             0.15,
                             32);

    // Linear velocity from wheel speed sensors received over CAN
    VSL_.init_add_stream_def(CAN_WS_Velocity_,
                             set_validity(true, false, false),
                             set_validity(false, false, false),
                             sumet_state_estimator::VelPrimary,
                             0.1,
                             32);

    // Setup Angular Velocity Streams
    VSL_.init_add_stream_def(Microstrain_,
                             set_validity(false, false, false),
                             set_validity(true, true, true),
                             sumet_state_estimator::VelPrimary,
                             // changed from 0.05 for testing with recorded data
                             0.1,
                             32);

    VSL_.init_add_stream_def(DSP3000_,
                             set_validity(false, false, false),
                             set_validity(false, false, true),
                             sumet_state_estimator::VelOnly,
                             0.05,
                             128);

    VSL_.init_add_stream_def(CAN_Angular_,
                             set_validity(false, false, false),
                             set_validity(false, false, true),
                             sumet_state_estimator::VelIgnore,
                             0.15,
                             32);

    // Yaw rate as calculated from the wheel speed sensors
    VSL_.init_add_stream_def(CAN_WS_Angular_,
                             set_validity(false, false, false),
                             set_validity(false, false, true),
                             sumet_state_estimator::VelSecondary,
                             0.1,
                             32);
  }

  /**
   * @brief  A brief method of creating a validity vector
   */
  bvec set_validity(bool b1 = false, bool b2 = false, bool b3 = false)
  {
    bvec vec;
    vec.push_back(b1);
    vec.push_back(b2);
    vec.push_back(b3);
    return vec;
  }

  /**
   * @brief       Initializes the node
   *
   */
  void init_node()
  {
    auto& pnh = nh_;//getPrivateNodeHandle();
    // Read in parameters

    // Initial latitude (for before we get a GPS message, default is for SwRI
    // campus 29.44 deg N)
    swri::param(pnh,"initial_latitude",
              cur_latitude_,
              29.44);

    swri::param(pnh,"verbose",
              verbose_,
              false);

    swri::param(pnh,"timeout_warning",
              timeout_warning_,
              1.0);

    swri::param(pnh,"require_wheel_encoders",
              require_wheel_encoders_,
              true);

    swri::param(pnh,"gyro_bias_estimation_time",
              gyro_bias_estimation_time_,
              15.0);

    // gyro_error_threshold_ enabled only for values above 0.0
    swri::param(pnh,"gyro_error_threshold",
              gyro_error_threshold_,
              -1.0);

    swri::param(pnh,"sampling_rate",
        sampling_rate_,
        100.0);

    swri::param(pnh,"rear_passenger_ws_cal_factor",
        rear_passenger_ws_cal_factor_,
        1.0);

    swri::param(pnh,"rear_driver_ws_cal_factor",
        rear_driver_ws_cal_factor_,
        1.0);

    swri::param(pnh,"longitudinal_velocity_timeout",
        long_vel_timeout_,
        0.75);

    swri::param(pnh,"yaw_rate_timeout",
        yaw_rate_timeout_,
        0.5);

    swri::param(pnh, "ignore_visual_odometry_diagnostic",
        ignore_visual_odometry_diagnostic_,
        true);

    // HMMWV vehicle track is approximately 72" (1.829 m) from wheel centers
    // MRZR vehicle track is approximately 51" (1.30 m) from wheel centers
    // Rubicon vehicle track is approximately 65.5" (1.66 m) from wheel centers
    swri::param(pnh,"track_width", track_width_, 1.829);

    swri::param(pnh,"wheel_slip_compensation_ratio",
        wheel_slip_compensation_ratio_,
        0.0);

    DT_ = 1.0 / sampling_rate_;

    setup_velocity_stream_list();
    local_earth_rate_ = (sumet_util::_earth_rate *
                         sumet_util::MathUtil::sind(cur_latitude_));

    last_twist_.header.stamp = rclcpp::Time(0,1);//ros::TIME_MIN;

    // Average 15 seconds of data from the gyro to compute the bias
    gyro_bias_.initialize(1000*gyro_bias_estimation_time_, nh_->now());

    init_diagnostics();
  }

  void setup_output_timer()
  {
    output_timer_ = create_timer(
      DT_,
      std::bind(&TwistAggregator::TimerCallback,
      this));
  }

  void LongitudinalVelocityDiagnostic(
      diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    double elapsed_time = swri::toSec(nh_->now() -
        last_longitudinal_vel_update_);


    if (elapsed_time < long_vel_timeout_)
    {
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK,
          "Longitudinal velocity is up to date in twist aggregator");
    }
    else
    {
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
          "Longitudinal velocity is stale");
    }
    status.add("Time since last longitudinal velocity update", elapsed_time);
  }

  void YawRateDiagnostic(diagnostic_updater::DiagnosticStatusWrapper& status)  // NOLINT
  {
    double elapsed_time = swri::toSec(last_yaw_rate_update_ -
        nh_->now());

    if (elapsed_time < yaw_rate_timeout_)
    {
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK,
          "Yaw rate is up to date in twist aggregator");
    }
    else
    {
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
          "Yaw rate is stale");
    }
    status.add("Time since last yaw rate update", elapsed_time);
  }

  void communicationDiagnostic(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    status.summary(diagnostic_updater::DiagnosticStatusWrapper::OK, "No errors reported.");
    Microstrain_Subscriber_.appendDiagnostics(status, "Microstrain",
                                              swri::Subscriber::DIAG_MOST);
    Novatel_Subscriber_.appendDiagnostics(status, "Novatel",
                                          swri::Subscriber::DIAG_MOST);
    Gyro_Subscriber_.appendDiagnostics(status, "Gyro",
                                       swri::Subscriber::DIAG_MOST);
    Wheel_Speed_Subscriber_.appendDiagnostics(status, "Wheel Speed",
                                              swri::Subscriber::DIAG_MOST);
    if (!ignore_visual_odometry_diagnostic_)
    {
      Visual_Odom_Msg_Subscriber_.appendDiagnostics(status, "Visual Odometry",
                                                    swri::Subscriber::DIAG_MOST);
    }
    transmission_sense_sub_.appendDiagnostics(status, "Transmission",
                                              swri::Subscriber::DIAG_MOST);
  }

  void RunDiagnostics()
  {
    diagnostic_updater_->update();
  }

  void init_diagnostics()
  {
    diagnostic_updater_->setHardwareID("twist_aggregator");

    diagnostic_updater_->add("Longitudinal Velocity Status",
        this,
        &TwistAggregator::LongitudinalVelocityDiagnostic);

    diagnostic_updater_->add("Yaw Rate Status",
        this,
        &TwistAggregator::YawRateDiagnostic);

    diagnostic_updater_->add("Communications",
                            this,
                            &TwistAggregator::communicationDiagnostic);

    diagnostic_timer_ =
        create_timer(1.0,
            std::bind(&TwistAggregator::RunDiagnostics,
            this));
  }
};
}  // namespace sumet_state_estimator

// Register nodelet plugin
#include "class_loader/class_loader_register_macro.h"

CLASS_LOADER_REGISTER_CLASS(sumet_state_estimator::TwistAggregator, swri::Node)
