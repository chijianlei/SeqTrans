// *****************************************************************************
//
// Copyright (C) 2017 All Right Reserved, Southwest Research Institute® (SwRI®)
//
// THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
// KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
// PARTICULAR PURPOSE.
//
// *****************************************************************************

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <diagnostic_updater/diagnostic_updater.h>

#include <marti_common_msgs/Float32Stamped.h>
#include <rtk_steering_calibration/ackermann.h>

// Alias type for easier access to DiagnosticStatus enumerations.
typedef diagnostic_msgs::DiagnosticStatus DS;
namespace du = diagnostic_updater;
namespace mcm = marti_common_msgs;

namespace rtk_steering_calibration
{
class SteeringCalibrationNodelet : public nodelet::Nodelet
{
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  boost::shared_ptr<du::Updater> diagnostic_updater_;
  ros::Timer diagnostic_timer_;

  ros::WallTimer init_timer_;

  ros::Subscriber steering_sense_sub_;
  ros::Subscriber curvature_setpoint_sub_;

  ros::Publisher steering_setpoint_pub_;
  ros::Publisher curvature_sense_pub_;

  ros::Subscriber parameters_sub_;
  ros::Publisher parameters_pub_;

  AckermannParameters cal_;

  void onInit()
  {
    nh_ = getNodeHandle();
    pnh_ = getPrivateNodeHandle();

    // create a one-shot timer to initialize everything after a brief
    // pause so that ros has time to connect to /rosout and we don't
    // drop errors/info during initialization.
    double initialization_delay = 1.0;
    pnh_.param("initialization_delay_s", initialization_delay, 1.0);
    init_timer_ = nh_.createWallTimer(ros::WallDuration(initialization_delay),
                                      &SteeringCalibrationNodelet::initialize,
                                      this,
                                      true);
  }

  void initialize(const ros::WallTimerEvent &)
  {
    // read the initial configuration off the parameter server
    double temp;
    pnh_.param("zero_curvature_steering", temp, 0.5);
    cal_.zero_curvature_steering = temp;
    pnh_.param("center_slope", temp, 0.2);
    cal_.center_slope = temp;
    pnh_.param("distortion_factor", temp, 4.0);
    cal_.distortion_factor = temp;

    // Steering -> Curvature conversion support
    steering_sense_sub_ = nh_.subscribe("steering_sense",2,&SteeringCalibrationNodelet::handleSteeringSense, this);
    curvature_sense_pub_ = nh_.advertise<mcm::Float32Stamped>("curvature_sense", 2);

    // Curvature -> Steering conversion support
    curvature_setpoint_sub_ = nh_.subscribe("curvature_setpoint", 2, &SteeringCalibrationNodelet::handleCurvatureSetpoint, this);
    steering_setpoint_pub_ = nh_.advertise<mcm::Float32Stamped>("steering_setpoint", 2);

    // Diagnostics
    diagnostic_timer_ = nh_.createTimer(ros::Duration(1.0),
                                    &SteeringCalibrationNodelet::handleDiagnosticTimer,
                                    this);
    diagnostic_updater_ = boost::make_shared<du::Updater>(nh_, pnh_, getName());
    diagnostic_updater_->setHardwareID("none");
    diagnostic_updater_->add(
      "Steering Calibration", this,
      &SteeringCalibrationNodelet::updateDiagnostics);
  }

  void handleDiagnosticTimer(const ros::TimerEvent &)
  {
    diagnostic_updater_->update();
  }

  void handleSteeringSense(const mcm::Float32StampedConstPtr &in_msg)
  {
    mcm::Float32StampedPtr out_msg = boost::make_shared<mcm::Float32Stamped>();
    out_msg->header.stamp = in_msg->header.stamp;
    curvaturesFromSteerings(&out_msg->value, &cal_, &in_msg->value, 1);
    ROS_INFO("got %f from ss, publishing %f to cs",&in_msg->value,&out_msg->value);
    curvature_sense_pub_.publish(out_msg);
  }

  void handleCurvatureSetpoint(const mcm::Float32StampedConstPtr &in_msg)
  {
    mcm::Float32StampedPtr out_msg = boost::make_shared<mcm::Float32Stamped>();
    out_msg->header.stamp = in_msg->header.stamp;
    steeringsFromCurvatures(&out_msg->value, &cal_, &in_msg->value, 1);
    ROS_INFO("got %f from csp, publishing %f to ssp",&in_msg->value,&out_msg->value);
    steering_setpoint_pub_.publish(out_msg);
  }

  void updateDiagnostics(du::DiagnosticStatusWrapper& status)
  {
    status.summary(DS::OK, "No errors reported.");
  }
};  // class SteeringConversionNodelet

boost::shared_ptr<nodelet::Nodelet> createSteeringCalibrationNodelet()
{
  return boost::make_shared<SteeringCalibrationNodelet>();
}
}  // namespace rtk_steering_calibration

// Register nodelet plugin
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(
  rtk_steering_calibration::SteeringCalibrationNodelet,
  nodelet::Nodelet)
