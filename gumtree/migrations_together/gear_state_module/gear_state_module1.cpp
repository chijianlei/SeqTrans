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
#include <sumet_low_level_controller/gear_state_module.h>

#include <string>

//#include <sumet_low_level_controller/local_path_module.h>
#include <sumet_low_level_controller/odometry_module.h>

#include <sumet_nav_msgs/DbwGear.h>
#include <marti_common_msgs/StringStamped.h>

#include <marti_dbw_msgs/constants.h>
#include <marti_dbw_msgs/TransmissionFeedback.h>

namespace snm = sumet_nav_msgs;
namespace mcm = marti_common_msgs;
namespace mdm = marti_dbw_msgs;

namespace sumet_low_level_controller
{
GearStateModule::GearStateModule()
  :
  min_gear_request_period_s_(-1.0),
  last_gear_request_time_(ros::Time::now()),
  last_requested_gear_(snm::DbwGear::UNKNOWN),
  desired_gear_(snm::DbwGear::PARK),
  current_gear_(snm::DbwGear::UNKNOWN),
  reported_gear_(snm::DbwGear::UNKNOWN)
{
}

GearStateModule::~GearStateModule()
{
}

void GearStateModule::Reconfigure()
{
  ros::NodeHandle mnh = GetModuleNodeHandle();

  mnh.param("min_gear_request_period_s", min_gear_request_period_s_, -1.0);
  ROS_INFO("%s: min_gear_request_period_s = %f",
           GetName().c_str(), min_gear_request_period_s_);

  std::string initial_gear_mode_str = "auto";
  mnh.getParam("initial_gear_mode", initial_gear_mode_str);
  ROS_INFO("%s: initial_gear_mode = %s",
           GetName().c_str(), initial_gear_mode_str.c_str());

  int32_t initial_gear_mode;
  if (initial_gear_mode_str == "park")
    initial_gear_mode = snm::DbwGear::PARK;
  else if (initial_gear_mode_str == "neutral")
    initial_gear_mode = snm::DbwGear::NEUTRAL;
  else if (initial_gear_mode_str == "reverse")
    initial_gear_mode = snm::DbwGear::REVERSE;
  else if (initial_gear_mode_str == "drive2")
    initial_gear_mode = snm::DbwGear::DRIVE2;
  else if (initial_gear_mode_str == "drive")
    initial_gear_mode = snm::DbwGear::DRIVE;
  else if (initial_gear_mode_str == "auto")
    initial_gear_mode = snm::DbwGear::AUTOMATIC;
  else
  {
    ROS_ERROR("%s: Unrecognized initial_gear_mode: %s",
              GetName().c_str(), initial_gear_mode_str.c_str());
    initial_gear_mode = snm::DbwGear::PARK;
  }

  if (!gear_mode_set_)
    gear_mode_ = initial_gear_mode;

  std::string automatic_forward_gear_str = "high";
  mnh.getParam("automatic_forward_gear", automatic_forward_gear_str);
  ROS_INFO("%s: automatic_forward_gear = %s",
           GetName().c_str(), automatic_forward_gear_str.c_str());

  if (automatic_forward_gear_str == "low")
    automatic_forward_gear_ = snm::DbwGear::DRIVE2;
  else if (automatic_forward_gear_str == "high")
    automatic_forward_gear_ = snm::DbwGear::DRIVE;
  else
  {
    ROS_ERROR("%s: Invalid automatic_forward_gear: %s",
              GetName().c_str(), automatic_forward_gear_str.c_str());
    automatic_forward_gear_ = snm::DbwGear::DRIVE;
  }

  std::string automatic_reverse_gear_str = "reverse";
  mnh.getParam("automatic_reverse_gear", automatic_reverse_gear_str);
  ROS_INFO("%s: automatic_reverse_gear = %s",
           GetName().c_str(), automatic_reverse_gear_str.c_str());

  if (automatic_reverse_gear_str == "reverse")
    automatic_reverse_gear_ = snm::DbwGear::REVERSE;
  else
  {
    ROS_ERROR("%s: Invalid automatic_reverse_gear: %s",
              GetName().c_str(), automatic_reverse_gear_str.c_str());
    automatic_reverse_gear_ = snm::DbwGear::REVERSE;
  }

  mnh.param("auto_gear_in_reverse_timeout_s", auto_gear_in_reverse_timeout_s_, -1.0);
  ROS_INFO("%s: auto_gear_in_reverse_timeout_s = %f",
             GetName().c_str(), auto_gear_in_reverse_timeout_s_);

}

void GearStateModule::Update()
{
  UpdateDesiredGear();
  CheckGearForDirection();

  if (desired_gear_ != current_gear_)
    AddStopWithBrake("Current gear is not desired gear.");

  ros::Time now = ros::Time::now();
  HandleGearAssignment(now);

}

void GearStateModule::UpdateDesiredGear()
{
  if (gear_mode_ != snm::DbwGear::AUTOMATIC) {
    SetDesiredGear(gear_mode_);
    return;
  }

  //in automatic

  //removed timeout check

  if (auto_gear_in_reverse_ && //not nullptr
      auto_gear_in_reverse_->value) {
    SetDesiredGear(automatic_reverse_gear_);
  } else {
    SetDesiredGear(automatic_forward_gear_);
  }
}

void GearStateModule::CheckGearForDirection()
{
  int32_t current_gear = currentGear();

  if (current_gear == snm::DbwGear::UNKNOWN)
  {
    AddStop("Current gear is unknown.");
  }
  else if (current_gear == snm::DbwGear::PARK)
  {
    // Do nothing.
  }
  else if (current_gear == snm::DbwGear::REVERSE)
  {
    // Do nothing.
  }
  else if (current_gear == snm::DbwGear::NEUTRAL)
  {
    // Do nothing.
  }
  else if (current_gear == snm::DbwGear::DRIVE ||
      current_gear == snm::DbwGear::DRIVE2)
  {
    // Do nothing.
  }

}

void GearStateModule::Initialize()
{
  ros::NodeHandle nh = GetNodeHandle();
  set_vehicle_gear_srv_ = nh.advertiseService(
    "set_vehicle_gear",
    &GearStateModule::SetVehicleGearService,
    this);
  
  gear_mode_set_ = false;

  transmission_input_pub_ = nh.advertise<mcm::StringStamped>(
    "transmission_input", 10, false);
  
  transmission_sense_sub_ = nh.subscribe(
    "transmission_sense", 10,
    &GearStateModule::handleTransmissionSense,
    this);
  
  auto_gear_in_reverse_sub_ = nh.subscribe(
    "auto_gear_in_reverse", 2,
    &GearStateModule::handleAutoGearInReverse,
    this);

  Reconfigure();
}

void GearStateModule::Shutdown()
{
  set_vehicle_gear_srv_.shutdown();
}

bool GearStateModule::SetVehicleGearService(
  snm::SetVehicleGear::Request &request,
  snm::SetVehicleGear::Response &response)
{
  switch (request.mode)
  {
  case snm::DbwGear::PARK:  // Intentional fall-through
  case snm::DbwGear::REVERSE:  // Intentional fall-through
  case snm::DbwGear::NEUTRAL:  // Intentional fall-through
  case snm::DbwGear::DRIVE:  // Intentional fall-through
  case snm::DbwGear::DRIVE2:  // Intentional fall-through
  case snm::DbwGear::AUTOMATIC:  // Intentional fall-through
    gear_mode_set_ = true;
    gear_mode_ = request.mode;
    response.result.success = true;
    response.result.message = "success";
    return true;
  default:
    response.result.success = false;
    response.result.message = "invalid gear mode";
    return true;
  }
}

int32_t GearStateModule::gearMode() const
{
  return gear_mode_;
}

int32_t GearStateModule::desiredGear() const
{
  return desired_gear_;
}

int32_t GearStateModule::currentGear() const
{
  return current_gear_;
}

bool GearStateModule::inPark() const
{
  if (currentGear() == snm::DbwGear::PARK)
    return true;
  else
    return false;
}

bool GearStateModule::inReverse() const
{
  if (currentGear() == snm::DbwGear::REVERSE)
    return true;
  else
    return false;
}

bool GearStateModule::inForward() const
{
  if (currentGear() == snm::DbwGear::DRIVE ||
      currentGear() == snm::DbwGear::DRIVE2)
    return true;
  else
    return false;
}

void GearStateModule::SetDesiredGear(int32_t gear)
{
  desired_gear_ = gear;
}

void GearStateModule::HandleGearAssignment(const ros::Time &now)
{
  if (desired_gear_ == reported_gear_)
  {
    ROS_INFO_THROTTLE(1.0, "%s: Don't publish transmission input. desired gear (%d) == reported gear (%d)",
        GetName().c_str(), desired_gear_, reported_gear_);
    return;
  }

  double request_age = (now - last_gear_request_time_).toSec();
  if (desired_gear_ == last_requested_gear_ &&
      min_gear_request_period_s_ > 0 &&
      request_age < min_gear_request_period_s_)
  {
    ROS_INFO("%s: Don't publish transmission input. request_age %f < %f",
        GetName().c_str(), request_age, min_gear_request_period_s_);
    return;
  }

  // Never change device states when ITO is active.  The
  // marti_dbw_interface will ignore these messages.  Doing it here
  // helps get more consistent behavior so that you send the request
  // immediately after going into robotic mode, instead of whenever
  // the repeat message comes around.
  if (!GetController()->roboticMode())
  {
    ROS_INFO_THROTTLE(1.0, "%s: Don't publish transmission input. not in robotic mode", GetName().c_str());
    return;
  }

  // Don't request a transmission gear shift unless vehicle is stopped
  if (!GetController()->GetOdometry()->VehicleStopState()) {
    ROS_INFO_THROTTLE(1.0, "%s: Don't publish transmission input. Vehicle not stopped.", GetName().c_str());
    return;
  }
  
  last_gear_request_time_ = ros::Time::now();
  last_requested_gear_ = desired_gear_;
  reported_gear_ = snm::DbwGear::UNKNOWN;

  mcm::StringStampedPtr msg = boost::make_shared<mcm::StringStamped>();
  msg->header.stamp = ros::Time::now();
  msg->value = newRangeFromOld(desired_gear_);
  transmission_input_pub_.publish(msg);
}

void GearStateModule::handleTransmissionSense(
  const marti_dbw_msgs::TransmissionFeedbackConstPtr &msg)
{
  reported_gear_ = oldRangeFromNew(msg->current_range);  
  if (!msg->stable) {
    current_gear_ = snm::DbwGear::UNKNOWN;
  } else {
    current_gear_ = reported_gear_;
  }
}

void GearStateModule::handleAutoGearInReverse(
        const marti_common_msgs::BoolStampedConstPtr &msg)
{
    auto_gear_in_reverse_ = msg;
}

int32_t GearStateModule::oldRangeFromNew(const std::string &range) const
{
  if (range == mdm::TRANS_PARK) {
    return snm::DbwGear::PARK;
  } else if (range == mdm::TRANS_REVERSE) {
    return snm::DbwGear::REVERSE;
  } else if (range == mdm::TRANS_NEUTRAL) {
    return snm::DbwGear::NEUTRAL;
  } else if (range == mdm::TRANS_DRIVE_LOW) {
    return snm::DbwGear::DRIVE2;
  } else if (range == mdm::TRANS_DRIVE_HIGH) {
    return snm::DbwGear::DRIVE;
  } else {
    return snm::DbwGear::UNKNOWN;
  }
}

const std::string & GearStateModule::newRangeFromOld(int32_t range) const
{
  if (range == snm::DbwGear::PARK) {
    return mdm::TRANS_PARK;
  } else if (range == snm::DbwGear::REVERSE) {
    return mdm::TRANS_REVERSE;
  } else if (range == snm::DbwGear::NEUTRAL) {
    return mdm::TRANS_NEUTRAL;
  } else if (range == snm::DbwGear::DRIVE2) {
    return mdm::TRANS_DRIVE_LOW;
  } else if (range == snm::DbwGear::DRIVE) {
    return mdm::TRANS_DRIVE_HIGH;
  } else {
    return mdm::TRANS_UNKNOWN;
  }
}
}  // namespace sumet_low_level_controller
