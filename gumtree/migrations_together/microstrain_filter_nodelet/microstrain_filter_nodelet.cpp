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
//

/**
 * \file
 *
 * This node generates an independent estimate of
 * attitude (specifically pitch and roll) using the accelerations measured by
 * IMU.  If available, the node will also use a TwistWithCovarianceStamped
 * message in conjunction with a simple vehicle model to improve the pitch and
 * roll estimates, particularly when undergoing long-term constant acceleration
 * events (driving on circular path, for example).  This node can be used with
 * any IMU, but was tested with a Microstrain 3DM-GX3-25.
 *   - <b>Subscribed Topics</b>
 *      - \e imu_in <tt>[sensor_msgs::Imu]</tt> - An IMU message.
 *      - \e VehicleVelocity6D <tt>[geometry_msgs::TwistWithCovarianceStamped]</tt> -
 *      A vehicle velocity message (in the form of a twist).  The
 *      sumet_velocity3D_aggregator_node publishes this message for the sumet
 *      system.
 *
 *   - <b>Published Topics</b>
 *      - \e imu_out <tt>[sensor_msgs::Imu]</tt> - A copy of the input
 *      imu/data messsage with revised pitch and roll in the orientation field.
 *
 *   - \b Parameters
 *      - \e legacy_mode <tt>[bool]</tt> - Set to true to output the data in the
 *      same frame (front-right-down) and format as original. Otherwise data is
 *      transoformed into and provided in vehicle frame (front-left-up). [true]
 *      - \e g_magnitude <tt>[double]</tt> - Magnitude of gravitational
 *      acceleration [9.8]
 *      - \e init_roll <tt>[double]</tt> - Roll offset from installed frame to
 *      vehicle frame according to RPY rotations. Transform from legacy to
 *      vehicle frame requires only roll to be set to 180.0 deg. [0.0]
 *      - \e init_pitch <tt>[double]</tt> - Pitch offset from installed frame to
 *      vehicle frame according to RPY rotations. Transform from legacy to
 *      vehicle frame requires pitch to be set to 0.0 deg. [0.0]
 *      - \e init_yaw <tt>[double]</tt> - Yaw offset from installed frame to
 *      vehicle frame according to RPY rotations. Transform from legacy to
 *      vehicle frame requires yaw to be set to 0.0 deg. [0.0]
 *      - \e base_var_scale <tt>[double]</tt> - Scale factor for variance of
 *      pitch and roll components of orientation. [1.0]
 *      - \e min_pitch_roll_variance <tt>[double]</tt> - The minimum variance
 *      that will be output for pitch and roll components of orientation under
 *      best conditions. [0.0004]
 *      - \e angular_rate_variance_scale <tt>[double]</tt> - A scale factor for
 *      variance of all three components of angular rate. [1.0]
 *      - \e wx_nominal_bias <tt>[double]</tt> - The "turn on" bias for wx
 *      angular rate before calibration. [0.0]
 *      - \e wy_nominal_bias <tt>[double]</tt> - The "turn on" bias for wy
 *      angular rate before calibration. [0.0]
 *      - \e wz_nominal_bias <tt>[double]</tt> - The "turn on" bias for wz
 *      angular rate before calibration. [0.0]
 *      - \e bias_est_num_samples <tt>[double]</tt> - The number of samples to
 *      use for bias estimation. [2000]
 *      - \e imu_freq <tt>[double]</tt> - The expected frequency in Hz of imu
 *      updates (used for diagnostics) [100.0]
 *      - \e velocity_freq <tt>[double]</tt> -  The expected frequency in Hz
 *      of velocity updates (used for diagnostics) [100.0]
 *      - \e max_latency <tt>[double]</tt> - The max time in seconds allowed for
 *      filter processing (used for diagnostics) [0.010]
 */

// Standard Libraries
#include <fstream>
#include <string>

#include <boost/shared_ptr.hpp>

// Boost Libraries
#include <boost/shared_ptr.hpp>

// ROS Libraries
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <tf/tf.h>
#include <nodelet/nodelet.h>
#include <diagnostic_updater/diagnostic_updater.h>

// SUMET Libraries
#include <sumet_state_estimator/BiasCalcClass.h>
#include <sumet_util/math_util.h>
#include <sumet_diagnostics/timing_monitor.h>
#include <diagnostic_updater/publisher.h>

// MARTI Libraries
#include <swri_transform_util/transform_util.h>

// SwRI Libraries
#include <swri_roscpp/subscriber.h>
#include <swri_roscpp/parameters.h>

namespace sumet_state_estimator
{
  /**
   * @brief An intermediary node for improving pitch and roll estimates from the
   *  Microstrain AHRS
   */
  class MicrostrainFilterNodelet : public nodelet::Nodelet
  {
    public:
      MicrostrainFilterNodelet():
        last_vel_update_(0),
        last_lat_accel_(0),
        last_long_accel_(0),
        last_linear_vel_(0),
        last_angular_vel_(0),
        last_imu_update_(0),
        stopped_time_(0.0),
        stop_time_(0),
        imu_to_vehicle_trans_(tf::Transform::getIdentity()),
        init_roll_(0),
        init_pitch_(0),
        init_yaw_(0),
        g_magnitude_(9.8),
        legacy_mode_(false),
        wxb0_(0.0),
        wyb0_(0.0),
        wzb0_(0.0),
        min_variance_(0.02),
        base_var_scale_(1.0),
        angular_rate_variance_scale_(1.0),
        max_interval1_(0),
        max_interval2_(0),
        max_vel_interval_(0),
        max_latency_(0),
        last_imu_msg_(new sensor_msgs::Imu)
      {
      }

      ~MicrostrainFilterNodelet()
      {
      }

    private:
      // Subscriptions
      swri::Subscriber Microstrain_Subscriber_;

      // If the velocity aggregator is running, we'll capture that data and
      // use it to improve Microstrain pitch and roll measurement performance.
      swri::Subscriber Velocity_Messages_Subscriber_;

      // Publishers
      ros::Publisher Revised_IMU_;

      // Storage for the latest measurements that we'll use
      ros::Time last_vel_update_;
      double last_lat_accel_;
      double last_long_accel_;
      double last_linear_vel_;
      double last_angular_vel_;
      ros::Time last_imu_update_;
      double stopped_time_;
      ros::Time stop_time_;

      BiasCalcClass wxb_;
      BiasCalcClass wyb_;
      BiasCalcClass wzb_;
      geometry_msgs::Vector3 cur_bias_;

      tf::Transform imu_to_vehicle_trans_;
      double init_roll_;
      double init_pitch_;
      double init_yaw_;

      // Parameters

      // The nominal magnitude of gravity (we can use this to increase
      // confidence) when the acceleration measurements are very close to this
      // value.
      double g_magnitude_;
      bool legacy_mode_;
      double wxb0_;
      double wyb0_;
      double wzb0_;
      geometry_msgs::Vector3 wzb_vec_;
      double min_variance_;
      double base_var_scale_;
      double angular_rate_variance_scale_;

      // Diagnostic Updater
      boost::shared_ptr<diagnostic_updater::Updater> updater_;
      double max_interval1_;
      double max_interval2_;
      double max_vel_interval_;
      double max_latency_;
      sensor_msgs::ImuPtr last_imu_msg_;

      // Timing monitors for subscribers
      sumet_diagnostics::TimingMonitor imu_monitor_;
      sumet_diagnostics::TimingMonitor vel_monitor_;

      // Timing monitors for output
      sumet_diagnostics::TimingMonitor output_monitor_;
      sumet_diagnostics::TimingMonitor latency_monitor_;

      ros::Timer diagnostics_timer_;

      // Used to delay initialization
      ros::WallTimer init_timer_;

      /**
       * @brief Main method for the nodelet
       *
       * Calls init_node(), subscribe_to_topics(), publish_topics(), and sets up
       * the diagnostic updater, then spins.
       */
      void onInit()
      {
        double initialization_delay = 1.0;
        swri::param(getPrivateNodeHandle(),"initialization_delay_s", initialization_delay, 1.0);
        init_timer_ = getNodeHandle().createWallTimer(ros::WallDuration(initialization_delay),
                                                      &MicrostrainFilterNodelet::initialize,
                                                      this,
                                                      true);
      }

      void initialize(const ros::WallTimerEvent&)
      {
        updater_ = boost::make_shared<diagnostic_updater::Updater>(
            getNodeHandle(), getPrivateNodeHandle(), getName());
        // Initialize the node
        init_node();

        // Publish topics
        advertise_topics();

        // Subscribe to topics
        subscribe_to_topics();

        // Set up the diagnostic updater
        setup_diagnostics();
      }

      /**
       * @brief Initializes the node
       */
      void init_node()
      {
        // Read parameters:
        ros::NodeHandle& pnh = getPrivateNodeHandle();

        swri::param(pnh,"legacy_mode", legacy_mode_, true);

        swri::param(pnh,"g_magnitude", g_magnitude_, 9.8);

        // Specify the installation orientation of the sensor relative to the
        // vehicle.
        swri::param(pnh,"init_roll", init_roll_, 0.0);
        swri::param(pnh,"init_pitch", init_pitch_, 0.0);
        swri::param(pnh,"init_yaw", init_yaw_, 0.0);
        init_roll_ *= sumet_util::_deg_2_rad;
        init_pitch_ *= sumet_util::_deg_2_rad;
        init_yaw_ *= sumet_util::_deg_2_rad;
        tf::Quaternion rpy;
        rpy.setRPY(init_roll_, init_pitch_, init_yaw_);
        imu_to_vehicle_trans_.setRotation(rpy);


        // Parameters to adjust variance values (for tuning convergence rates
        // and static errors for attitude)
        swri::param(pnh,"base_var_scale", base_var_scale_, 1.0);
        swri::param(pnh,"min_pitch_roll_variance", min_variance_, 0.0004);
        swri::param(pnh,"angular_rate_variance_scale",
            angular_rate_variance_scale_,
            1.0);

        // Setup bias offset and estimation parameters
        swri::param(pnh,"wx_nominal_bias", wxb0_, 0.0);
        swri::param(pnh,"wy_nominal_bias", wyb0_, 0.0);
        swri::param(pnh,"wz_nominal_bias", wzb0_, 0.0);
        wzb_vec_.x = wxb0_;
        wzb_vec_.y = wyb0_;
        wzb_vec_.z = wzb0_;

        // The microstrain outputs data at 100Hz so to capture 20 seconds worth
        // of data as a default, 2000 samples is required.
        int32_t bias_est_num_samples = 2000;
        swri::param(pnh,
            "bias_est_num_samples",
            bias_est_num_samples,
            bias_est_num_samples);

        wxb_.initialize(bias_est_num_samples);
        wyb_.initialize(bias_est_num_samples);
        wzb_.initialize(bias_est_num_samples);

        double velocity_freq, imu_freq;

        swri::param(pnh,"imu_freq", imu_freq, 100.0);
        swri::param(pnh,"velocity_freq", velocity_freq, 100.0);

        max_interval1_ = 0.2;
        max_interval2_ = 1.0;
        max_vel_interval_ = 3.0 / velocity_freq;
        swri::param(pnh,"max_latency", max_latency_, 0.10);

        last_lat_accel_ = 0.0;
        last_long_accel_ = 0.0;
        last_vel_update_ = ros::TIME_MIN;
        last_imu_update_ = ros::TIME_MIN;
      }

      void subscribe_to_topics()
      {
        ros::NodeHandle& nh = getNodeHandle();

        Microstrain_Subscriber_ = swri::Subscriber( nh,"imu_in", 5,
            &MicrostrainFilterNodelet::Microstrain_cb, this);
        imu_monitor_.Begin(ros::Time::now().toNSec());

        Velocity_Messages_Subscriber_ = swri::Subscriber( nh,"VehicleVelocity6D", 5,
            &MicrostrainFilterNodelet::Velocity_cb, this);
        vel_monitor_.Begin(ros::Time::now().toNSec());
      }

      /**
       * @brief Advertises the publisher for revised IMU data and starts the
       * timing monitor for the output.
       *
       */
      void advertise_topics()
      {
        ros::NodeHandle& nh = getNodeHandle();
        Revised_IMU_ = nh.advertise<sensor_msgs::Imu>("imu_out", 100);
        output_monitor_.Begin(ros::Time::now().toNSec());
      }


      /**
       * @brief Extracts velocity, acceleration and orientation data from
       * Microstrain, modifies it, and republishes the relevant bits.
       *
       * Performs frame rotation if necessary and calculates more accurate
       * orientation, then publishes the results.
       *
       * @param[in]   msg           Imu message from the Microstrain node
       */
      void Microstrain_cb(const sensor_msgs::ImuConstPtr& msg)
      {
        imu_monitor_.EndBegin(msg->header.stamp.toNSec());
        latency_monitor_.Begin(msg->header.stamp.toNSec());
        last_imu_update_ = msg->header.stamp;

        sensor_msgs::ImuPtr rev_imu(new sensor_msgs::Imu(*msg));

        handle_biases(rev_imu);

        transform_measurement(rev_imu, imu_to_vehicle_trans_);

        remove_non_gravity_accels(rev_imu);

        estimate_pitch_and_roll(rev_imu);

        if (legacy_mode_)
        {
          // The legacy frame for IMU is forward-right-down (x-y-z).  Although
          // the angular rates and accelerations can be transformed directly,
          // the orientation needs a little more work.
          tf::Quaternion Q;
          Q.setRPY(sumet_util::_pi, 0.0, 0.0);
          tf::Transform T(Q);
          transform_measurement(rev_imu, T);
          tf::Quaternion Qtmp(rev_imu->orientation.x,
              rev_imu->orientation.y,
              rev_imu->orientation.z,
              rev_imu->orientation.w);
          double r;
          double p;
          double y;
          tf::Transform(Qtmp).getBasis().getRPY(r, p, y);
          Qtmp.setRPY(r, -p, -y);
          rev_imu->orientation.x = Qtmp.getX();
          rev_imu->orientation.y = Qtmp.getY();
          rev_imu->orientation.z = Qtmp.getZ();
          rev_imu->orientation.w = Qtmp.getW();
        }

        Revised_IMU_.publish(rev_imu);
        latency_monitor_.End(ros::Time::now().toNSec());
        output_monitor_.EndBegin(ros::Time::now().toNSec());
      }

      void handle_biases(sensor_msgs::ImuPtr& msg)
      {
        // Subtract nominal sensor biases:
        msg->angular_velocity.x -= wzb_vec_.x;
        msg->angular_velocity.y -= wzb_vec_.y;
        msg->angular_velocity.z -= wzb_vec_.z;


        // Check to see whether we're stopped to add
        if (stopped_time_ > 5.0)
        {
          wxb_.load_new_data(msg->angular_velocity.x, true);
          wyb_.load_new_data(msg->angular_velocity.y, true);
          wzb_.load_new_data(msg->angular_velocity.z, true);
        }

        cur_bias_.x = wxb_.get_current_bias();
        cur_bias_.y = wyb_.get_current_bias(),
        cur_bias_.z = wzb_.get_current_bias();

        msg->angular_velocity.x -= cur_bias_.x;
        msg->angular_velocity.y -= cur_bias_.y;
        msg->angular_velocity.z -= cur_bias_.z;
      }

      void transform_measurement(sensor_msgs::ImuPtr& msg,
          const tf::Transform& T)
      {
        // Snap the rotation to be axis aligned for processing the covariances
        // cleanly.  This is to account for the fact that the covariance
        // matrices contain special flag values that can't be blended which
        // would occur for a non-right-angle rotation.
        tf::Matrix3x3 aligned_rotation(
        swri_transform_util::SnapToRightAngle(T.getRotation()));

        // First rotate the measurements
        tf::Vector3 W(msg->angular_velocity.x,
            msg->angular_velocity.y,
            msg->angular_velocity.z);

        tf::Vector3 A(msg->linear_acceleration.x,
            msg->linear_acceleration.y,
            msg->linear_acceleration.z);

        tf::Quaternion Q(msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w);

    //    NODELET_ERROR("wz before = (%g)", W.getZ());
        W = T * W;
        A = T * A;
        Q = T * Q;
    //    NODELET_ERROR("wz after = (%g)", W.getZ());


        msg->angular_velocity.x = W.getX();
        msg->angular_velocity.y = W.getY();
        msg->angular_velocity.z = W.getZ();

        msg->linear_acceleration.x = A.getX();
        msg->linear_acceleration.y = A.getY();
        msg->linear_acceleration.z = A.getZ();

        msg->orientation.x = Q.getX();
        msg->orientation.y = Q.getY();
        msg->orientation.z = Q.getZ();
        msg->orientation.w = Q.getW();

        // Now rotate the covariances
        tf::Matrix3x3 Cw = swri_transform_util::Get3x3Cov(
            msg->angular_velocity_covariance);
        tf::Matrix3x3 Ca = swri_transform_util::Get3x3Cov(
            msg->linear_acceleration_covariance);
        tf::Matrix3x3 Cq = swri_transform_util::Get3x3Cov(
            msg->orientation_covariance);

        tf::Matrix3x3 Cw_rot = aligned_rotation.transposeTimes(Cw)*aligned_rotation;  // NOLINT
        tf::Matrix3x3 Ca_rot = aligned_rotation.transposeTimes(Ca)*aligned_rotation;  // NOLINT
        tf::Matrix3x3 Cq_rot = aligned_rotation.transposeTimes(Cq)*aligned_rotation;  // NOLINT

        swri_transform_util::Set3x3Cov(Cw_rot, msg->angular_velocity_covariance);
        swri_transform_util::Set3x3Cov(Ca_rot, msg->linear_acceleration_covariance);
        swri_transform_util::Set3x3Cov(Cq_rot, msg->orientation_covariance);

      }

      void remove_non_gravity_accels(sensor_msgs::ImuPtr& msg)
      {
        if ((msg->header.stamp - last_vel_update_).toSec() > 0.15)
        {
          // Don't use the acceleration estimates if source velocity data is
          // stale
          return;
        }
        msg->linear_acceleration.y -= last_lat_accel_;
        msg->linear_acceleration.x -= last_long_accel_;
      }

      void update_covariance(sensor_msgs::ImuPtr& msg)
      {
        double cur_var = min_variance_;
        double var_scale = base_var_scale_;

        // Scale the variance upwards when we are moving by 10., and downwards
        // to a minimum of 1.0 2 seconds after we've stopped
        var_scale = std::max(1.0 / (0.9 *(1.0 / 2.0)*stopped_time_ + 0.1), 1.0);

        tf::Vector3 measured_accel(msg->linear_acceleration.x,
            msg->linear_acceleration.y,
            msg->linear_acceleration.z);

        // Scale the variance quadratically as the magnitude of measured
        // acceleration varies from gravitational acceleration
        var_scale *= std::pow(5.0 * std::abs(g_magnitude_ - measured_accel.length()), 2.0) + 1.0;  // NOLINT

        cur_var *= var_scale;

        setCov3x3DiagVals(msg->orientation_covariance, cur_var,
            cur_var, sumet_util::_large_variance);

        // Scale angular rate covariances
        double v1;
        double v2;
        double v3;
        getCov3x3DiagVals(msg->angular_velocity_covariance, v1, v2, v3);
        setCov3x3DiagVals(msg->angular_velocity_covariance,
            angular_rate_variance_scale_*v1,
            angular_rate_variance_scale_*v2,
            angular_rate_variance_scale_*v3);
      }


      void estimate_pitch_and_roll(sensor_msgs::ImuPtr& msg)
      {
        tf::Vector3 g1(0.0, 0.0, 1.0);  // gravity unit vector
        tf::Vector3 measured_accel(
            msg->linear_acceleration.x,
            msg->linear_acceleration.y,
            msg->linear_acceleration.z);

        tf::Vector3 g2(measured_accel);
        g2.normalize();  // measured accel unit vector

        // Get the rotation axis
        tf::Vector3 rot_axis = g2.cross(g1);

        // compute the rotation angle:  |g1 x g2| = |g1|*|g2|*sin(theta)
        double angle = std::asin(rot_axis.length());
        rot_axis.normalize();

        tf::Quaternion Q_est;
        bool flipped = (g1.dot(g2) < 0);
        if (flipped)
        {
          double offset = sumet_util::_pi;
          Q_est.setRotation(rot_axis, offset - angle);
        }
        else
        {
          Q_est.setRotation(rot_axis, angle);
        }

        // Check to make sure we got the rotation correct:
        tf::Transform T(Q_est.inverse());
        tf::Vector3 test_vec2 = T * g1;

        double err_from_rotation = (test_vec2 - g2).length();
        if (err_from_rotation > 0.0001)
        {
          setCov3x3DiagVals(
              msg->orientation_covariance,
              sumet_util::_large_variance,
              sumet_util::_large_variance,
              sumet_util::_large_variance);
        }
        else
        {
          update_covariance(msg);
        }

        msg->orientation.x = Q_est.x();
        msg->orientation.y = Q_est.y();
        msg->orientation.z = Q_est.z();
        msg->orientation.w = Q_est.w();
      }

      /**
       * @brief Callback to store velocity messages
       *
       * Estimates the lateral acceleration from the rotational velocity and.
       *
       * @param[in]   msg   TwistWithCovarianceStamped message containing the 6D
       *                    velocity
       */
      void Velocity_cb(
          const geometry_msgs::TwistWithCovarianceStampedConstPtr &msg)
      {
        // Use linear and angular velocity to estimate the longitudinal and
        // lateral acceleration -- (NOTE: currently longitudinal acceleration is
        // not being estimated).

        // Update timing monitor for this subscriber
        vel_monitor_.EndBegin(msg->header.stamp.toNSec());

        double linear_vel = msg->twist.twist.linear.x;
        double angular_vel = msg->twist.twist.angular.z;

        last_vel_update_ = msg->header.stamp;

        // Estimate the lateral acceleration
        last_linear_vel_ = linear_vel;
        last_angular_vel_ = angular_vel;

        // Note that this gives the proper sign in the vehicle frame, but will
        // be opposite in the usual IMU frame -- we'll handle the conversion
        // when we use it.
        last_lat_accel_ = linear_vel * angular_vel;

        if (stopped_time_ > 0.0 && linear_vel == 0.0)
        {
          stopped_time_ = (ros::Time::now() - stop_time_).toSec();
        }
        else if (stopped_time_ == 0.0 && linear_vel == 0.0)
        {
          stop_time_ = msg->header.stamp;
          stopped_time_ = 0.000001;
        }
        else
        {
          stopped_time_ = 0.0;
        }

        updater_->update();
      }

      void setup_diagnostics()
      {
        updater_->setHardwareID("Microstrain Filter");
        updater_->add("IMU Data",
            this, &MicrostrainFilterNodelet::imu_diagnostic_cb);
        updater_->add("Velocity Data",
            this, &MicrostrainFilterNodelet::velocity_diagnostic_cb);
        updater_->add("IMU Bias Data",
            this, &MicrostrainFilterNodelet::bias_diagnostic_cb);
        updater_->add("Filter Output",
            this, &MicrostrainFilterNodelet::output_diagnostic_cb);
        updater_->add("Communications",
            this, &MicrostrainFilterNodelet::communicationDiagnostic);
        diagnostics_timer_ = getNodeHandle().createTimer(ros::Duration(1),
            &MicrostrainFilterNodelet::diagnostic_update_cb, this);
      }

      /**
       * @brief Wrapper for diagnostic update function, run on a 1Hz timer
       *
       *  This is important for when no messages are received (receiving either
       *  type of message will also trigger update()).
       *
       * @param[in] const ros::TimerEvent& event Required by ROS Timer event
       *                                         (unused)
       */
      void diagnostic_update_cb(const ros::TimerEvent& event)
      {
        // This is currently on a 1 second timer, so we'll force an update to
        // ensure that it gets run each call (normal update skips updates if
        // elapsed time is less than one second since the last update).
        updater_->force_update();
      }

      /**
       * @brief Diagnostic callback to update diagnostics and reset timing
       *        monitors
       */
      void imu_diagnostic_cb(diagnostic_updater::DiagnosticStatusWrapper &stat)  // NOLINT
      {
        stat.add("Time since last IMU update (s)",
            (ros::Time::now() - last_imu_update_).toSec());
        stat.add("Last IMU Acceleration (x)",
            last_imu_msg_->linear_acceleration.x);
        stat.add("Last IMU Acceleration (y)",
            last_imu_msg_->linear_acceleration.y);
        stat.add("Last IMU Acceleration (z)",
            last_imu_msg_->linear_acceleration.z);
        stat.add("Last IMU Angular Velocity (x)",
            last_imu_msg_->angular_velocity.x);
        stat.add("Last IMU Angular Velocity (y)",
            last_imu_msg_->angular_velocity.y);
        stat.add("Last IMU Angular Velocity (z)",
            last_imu_msg_->angular_velocity.z);

        if (imu_monitor_.GetCount() > 0)
        {
          stat.add("Average update rate (Hz)",
              1.0 / imu_monitor_.GetAverageTimeSec());
          stat.add("Longest time between messages (s)",
              imu_monitor_.GetMaxTimeSec());
          stat.add("Shortest time between message (s)",
              imu_monitor_.GetMinTimeSec());
          stat.add("Messages since last update",
              imu_monitor_.GetCount());
          if (imu_monitor_.GetMaxTimeSec() > max_interval2_)
          {
            stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR,
                "IMU updates lost");
          }
          else if (imu_monitor_.GetMaxTimeSec() > max_interval1_)
          {
            stat.summary(diagnostic_msgs::DiagnosticStatus::WARN,
                "IMU updating slowly");
          }
          else
          {
            stat.summary(diagnostic_msgs::DiagnosticStatus::OK,
                "IMU OK");
          }
        }
        else
        {
          stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR,
              "No data from IMU");
        }
        imu_monitor_.Reset();
      }

      /**
       * @brief Diagnostic callback to update diagnostics and reset timing monitors
       */
      void output_diagnostic_cb(diagnostic_updater::DiagnosticStatusWrapper &stat)  // NOLINT
      {
        if (latency_monitor_.GetMaxTimeSec() > max_latency_)
          stat.summary(diagnostic_msgs::DiagnosticStatus::WARN,
              "Filter latency high");
        else if (output_monitor_.GetMaxTimeSec() > max_interval2_)
          stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR,
              "Filter output rate very low");
        else if (output_monitor_.GetMaxTimeSec() > max_interval1_)
          stat.summary(diagnostic_msgs::DiagnosticStatus::WARN,
              "Filter output rate low");
        else
          stat.summary(diagnostic_msgs::DiagnosticStatus::OK,
              "Filter OK");

        if (latency_monitor_.GetCount() > 0)
        {
          stat.add("Average filter processing time (s)",
              latency_monitor_.GetAverageTimeSec());
          stat.add("Longest filter processing time (s)",
              latency_monitor_.GetMaxTimeSec());
          stat.add("Shortest filter processing time (s)",
              latency_monitor_.GetMinTimeSec());
        }
        if (output_monitor_.GetCount() > 0)
        {
          stat.add("Average update rate (Hz)",
              1.0 / output_monitor_.GetAverageTimeSec());
          stat.add("Longest time between messages (s)",
              output_monitor_.GetMaxTimeSec());
          stat.add("Shortest time between message (s)",
              output_monitor_.GetMinTimeSec());
          stat.add("Messages since last update",
              output_monitor_.GetCount());
        }
        else
        {
          stat.summary(diagnostic_msgs::DiagnosticStatus::WARN,
              "No IMU messages published since last update");
        }

        output_monitor_.Reset();
        latency_monitor_.Reset();
      }

      void bias_diagnostic_cb(diagnostic_updater::DiagnosticStatusWrapper &stat)
      {
        double bias_age = wxb_.get_bias_age().toSec();
        stat.add("Bias Estimate Age", bias_age);
        stat.add("Current wx Bias Estimate", wxb_.get_current_bias());
        stat.add("Current wy Bias Estimate", wyb_.get_current_bias());
        stat.add("Current wz Bias Estimate", wzb_.get_current_bias());
        const double bias_calibration_threshold =
            250.0 * sumet_util::_deg_2_rad / 3600.0;
        if (std::abs(wxb_.get_current_bias()) > bias_calibration_threshold ||
            std::abs(wxb_.get_current_bias()) > bias_calibration_threshold ||
            std::abs(wxb_.get_current_bias()) > bias_calibration_threshold)
        {
         stat.add("Bias is too high. Recalibration may be required. "
             "Current Threshold", bias_calibration_threshold);
        }
        else
        {
          stat.add("Biases are within calibration threshold",
              bias_calibration_threshold);
        }

        if (bias_age <= 1800.0)
        {
          stat.summary(diagnostic_msgs::DiagnosticStatus::OK,
              "Bias estimate is current");
        }
        else if (bias_age > 24.0 * 3600.0)
        {
          stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR,
              "Bias estimate invalid");
        }
        else
        {
          stat.summary(diagnostic_msgs::DiagnosticStatus::WARN,
              "Bias estimate is stale");
        }
      }

      /**
       * @brief Diagnostic callback to update velocity diagnostics and reset timing monitors
       */
      void velocity_diagnostic_cb(diagnostic_updater::DiagnosticStatusWrapper &stat)  // NOLINT
      {
        stat.add("Time since last velocity update (s)",
            (ros::Time::now() - last_vel_update_).toSec());
        if (vel_monitor_.GetCount() > 0)
        {
          stat.add("Average update rate (Hz)",
              1.0 / vel_monitor_.GetAverageTimeSec());
          stat.add("Longest time between messages (s)",
              vel_monitor_.GetMaxTimeSec());
          stat.add("Shortest time between message (s)",
              vel_monitor_.GetMinTimeSec());
          stat.add("Last linear velocity (x)",
              last_linear_vel_);
          stat.add("Last angular velocity",
              last_angular_vel_);
          stat.add("Last longitudinal acceleration",
              last_long_accel_);
          stat.add("Last lateral acceleration",
              last_lat_accel_);

          if (vel_monitor_.GetMaxTimeSec() > max_vel_interval_)
          {
            stat.summary(diagnostic_msgs::DiagnosticStatus::WARN,
                "Velocity updating slowly");
          }
          else
          {
            stat.summary(diagnostic_msgs::DiagnosticStatus::OK,
                "Velocity OK");
          }
        }
        else
        {
          stat.summary(diagnostic_msgs::DiagnosticStatus::WARN,
              "No data from Velocity Estimator");
        }
        vel_monitor_.Reset();
      }

      void communicationDiagnostic(diagnostic_updater::DiagnosticStatusWrapper& status)
      {
        status.summary(diagnostic_updater::DiagnosticStatusWrapper::OK, "No errors reported.");
        Microstrain_Subscriber_.appendDiagnostics(status, "Microstrain",
                                                  swri::Subscriber::DIAG_MOST);
        Velocity_Messages_Subscriber_.appendDiagnostics(status, "Vehicle Velocity",
                                                  swri::Subscriber::DIAG_MOST);
      }

      void setCov3x3DiagVals(boost::array<double, 9>& cov_in,
          double v1, double v2, double v3) const
      {
        cov_in[0] = v1;
        cov_in[4] = v2;
        cov_in[8] = v3;
      }

      void getCov3x3DiagVals(const boost::array<double, 9>& cov_in,
          double& v1, double& v2, double& v3) const
      {
        v1 = cov_in[0];
        v2 = cov_in[4];
        v3 = cov_in[8];
      }
  };
}  // namespace sumet_state_estimator

// Register nodelet plugin
#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(sumet_state_estimator,
    microstrain_filter_nodelet,
    sumet_state_estimator::MicrostrainFilterNodelet,
    nodelet::Nodelet)

