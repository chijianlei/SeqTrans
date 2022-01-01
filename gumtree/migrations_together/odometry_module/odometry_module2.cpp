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

#include <sumet_low_level_controller/odometry_module.h>
#include <sumet_low_level_controller/gear_state_module.h>


#include <tf2/convert.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace sumet_low_level_controller
{
OdometryModule::OdometryModule()
  :
  timeout_s_(-1.0),
  odom_valid_(false),
  odom_received_(false),
  odom_error_("<UNINITIALIZED>"),
  vehicle_x_(-1010101.0),
  vehicle_y_(-1010101.0),
  vehicle_roll_(-1010101.0),
  vehicle_pitch_(-1010101.0),
  vehicle_yaw_(-1010101.0),
  vehicle_velocity_(-1010101.0)
{
}

OdometryModule::~OdometryModule()
{
}

void OdometryModule::Reconfigure()
{
  auto mnh = GetModuleNodeHandle();

  timeout_s_ = (get_sub_parameter(mnh, "timeout_s")).as_double();
  RCLCPP_INFO(mnh->get_logger(),
              "%s: timeout_s = %f", GetName().c_str(), timeout_s_);

  target_frame_ = (get_sub_parameter(mnh, "target_frame")).as_string();
  RCLCPP_INFO(mnh->get_logger(),
              "%s: target_frame = %s", GetName().c_str(), target_frame_.c_str());

  stopped_speed_threshold_ = (get_sub_parameter(mnh, "stopped_speed_threshold")).as_double();
  RCLCPP_INFO(mnh->get_logger(),
              "%s: stopped_speed_threshold = %f", GetName().c_str(), stopped_speed_threshold_);

  // Invalidate odometry in case target frame changed.
  odom_valid_ = false;
  UpdateOdomValid();
}

void OdometryModule::Update()
{
  UpdateOdomValid();

  if (!odom_valid_)
    AddStop(odom_error_);
}

void OdometryModule::Initialize()
{
  auto mnh = GetModuleNodeHandle();

  // declare parameters
  RCLCPP_INFO(mnh->get_logger(),"This is namespace '%s'",mnh->get_effective_namespace().c_str());

  declare_sub_parameter(mnh,"timeout_s",rclcpp::ParameterValue(-1.0));
  declare_sub_parameter(mnh,"target_frame", rclcpp::ParameterValue(std::string("/near_field")));
  declare_sub_parameter(mnh,"stopped_speed_threshold", rclcpp::ParameterValue(0.01));

  odom_sub_ = mnh->create_subscription<nav_msgs::msg::Odometry>(
    "/localization/near_field_odom",
    3,
    std::bind(&OdometryModule::HandleOdomMessage,this,std::placeholders::_1)
  );

  Reconfigure();
}

void OdometryModule::Shutdown()
{
  odom_sub_.reset(); // HACKY ros2 analog
}

void OdometryModule::HandleOdomMessage(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  odom_msg_ = *msg;
  odom_received_ = true;
  UpdateOdomValid();

  if (!odom_valid_)
    return;

  vehicle_x_ = msg->pose.pose.position.x;
  vehicle_y_ = msg->pose.pose.position.y;

  tf2::Quaternion bt_q;
  // extract from msg
  tf2::fromMsg(msg->pose.pose.orientation, bt_q);
  tf2::Matrix3x3(bt_q).getEulerYPR(vehicle_yaw_,
                                   vehicle_pitch_,
                                   vehicle_roll_);

  vehicle_velocity_ = msg->twist.twist.linear.x;

  vehicle_stop_state_ = std::abs(msg->twist.twist.linear.x) < stopped_speed_threshold_;
}

double OdometryModule::SpeedFromVelocity(double velocity)
{
  return 0.0;
  if (GetController()->GetGearState()->inReverse())
    return -velocity;
  else
    return velocity;
}

void OdometryModule::UpdateOdomValid()
{
  auto mnh = GetModuleNodeHandle(); // for clock
  if (!odom_received_)
  {
    odom_valid_ = false;
    odom_error_ = "No odometry messages received.";
    return;
  }
  //Check that the frame is within a / of the target_frame

  if (odom_msg_.header.frame_id.empty())
  {
    char buffer[1024];
    snprintf(buffer, sizeof(buffer),
             "Odometry frame_id is empty");
    odom_valid_ = false;
    odom_error_ = std::string(buffer);
    return;
  }
  bool valid_frame = (odom_msg_.header.frame_id == target_frame_
                      || odom_msg_.header.frame_id == "/" + target_frame_
                      || "/" + odom_msg_.header.frame_id == target_frame_);

  if(!valid_frame)
  {
    char buffer[1024];
    snprintf(buffer, sizeof(buffer),
             "Odometry frame_id (%s) is not target frame (%s)",
             odom_msg_.header.frame_id.c_str(),
             target_frame_.c_str());
    odom_valid_ = false;
    odom_error_ = std::string(buffer);
    return;
  }

  double age = (mnh->now() - odom_msg_.header.stamp).seconds(); // WARNING this calculation may not work in ROS 2
  if (timeout_s_ > 0 && age > timeout_s_)
  {
    char buffer[1024];
    snprintf(buffer, sizeof(buffer),
             "Odometry has timed out (%f > %f)",
             age, timeout_s_);
    odom_valid_ = false;
    odom_error_ = std::string(buffer);
    return;
  }

  odom_valid_ = true;
  odom_error_ = "no error";
}
}  // namespace sumet_low_level_controller
