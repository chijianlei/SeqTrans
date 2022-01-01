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

#include <sumet_low_level_controller/speed_command_module.h>
#include <sumet_low_level_controller/odometry_module.h>
#include <sumet_low_level_controller/gear_state_module.h>
#include <sumet_nav_msgs/msg/dbw_gear.hpp>

namespace sumet_low_level_controller
{
SpeedCommandModule::SpeedCommandModule()
  :
  timeout_s_(-1.0),
  speed_valid_(false),
  speed_error_("<UNINITIALIZED>")
{
}

SpeedCommandModule::~SpeedCommandModule()
{
}

void SpeedCommandModule::Reconfigure()
{
  auto mnh = GetModuleNodeHandle();

  timeout_s_ = (get_sub_parameter(mnh, "timeout_s")).as_double();
  RCLCPP_INFO(mnh->get_logger(),
              "%s: timeout_s = %f", GetName().c_str(), timeout_s_
  );

  //recheck if valid
  speed_valid_ = false;
  UpdateSpeedValid();
}

void SpeedCommandModule::Update()
{
  UpdateSpeedValid();

  if (!speed_valid_)
  {
    AddStop(speed_error_);
  }
}

void SpeedCommandModule::Initialize()
{
  auto nh = GetNodeHandle();
  speed_sub_ = nh->create_subscription<marti_common_msgs::msg::Float32Stamped>(
    "speed_setpoint", 2,
    std::bind(&SpeedCommandModule::handleSpeedMsg, this, std::placeholders::_1)
  );
  // declare parameter
  declare_sub_parameter(GetModuleNodeHandle(),"timeout_s",rclcpp::ParameterValue(timeout_s_));
  Reconfigure();
}

void SpeedCommandModule::Shutdown()
{
  speed_sub_.reset(); // HACKY ros2 analog
}

void SpeedCommandModule::UpdateSpeedValid()
{
  /// stub for removed diagnostics

  speed_valid_ = true;
  speed_error_ = "no error";
}

double SpeedCommandModule::calculateSpeedCommand(const bool stop_mode)
{
  double cmd_speed;

  int32_t current_gear = GetController()->GetGearState()->currentGear();
  bool valid_gear;
  if (current_gear == sumet_nav_msgs::msg::DbwGear::REVERSE ||
      current_gear == sumet_nav_msgs::msg::DbwGear::DRIVE ||
      current_gear == sumet_nav_msgs::msg::DbwGear::DRIVE2)
  {
    valid_gear = true;
  }
  else
  {
    valid_gear = false; //PARK
  }

  if(stop_mode) //gear state module adds stop when current gear is not desired gear
  {
    cmd_speed = 0.0;
//    ROS_WARN_THROTTLE(1.0, "Keep commanded speed at zero because in stop mode.");
  }
  else if (!valid_gear)
  {
    cmd_speed = 0.0;
//    ROS_WARN_THROTTLE(1.0, "Keep commanded speed at zero because invalid gear (%d).", current_gear);
  }
  else if (speed_valid_)
  {
    cmd_speed = speed_msg_->value;
  }
  else
  {
    cmd_speed = 0.0;
    auto mnh = GetModuleNodeHandle();
    RCLCPP_WARN(mnh->get_logger(), "Speed invalid!");
  }

  return cmd_speed;
}

void SpeedCommandModule::handleSpeedMsg(const marti_common_msgs::msg::Float32Stamped::SharedPtr msg)
{
    speed_msg_ = msg;
}

} //namespace
