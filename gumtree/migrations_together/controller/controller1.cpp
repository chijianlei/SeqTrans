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
#include <ros/ros.h>
#include <sumet_low_level_controller/controller.h>
#include <sumet_low_level_controller/controller_module.h>

#include <sumet_low_level_controller/curvature_command_module.h>
#include <sumet_low_level_controller/gear_state_module.h>
#include <sumet_low_level_controller/ignition_module.h>
#include <sumet_low_level_controller/mototron_status_module.h>
#include <sumet_low_level_controller/odometry_module.h>
#include <sumet_low_level_controller/speed_command_module.h>

#include <sumet_nav_msgs/DriveByWireState.h>

#include <marti_common_msgs/Float32Stamped.h>

namespace snm = sumet_nav_msgs;
namespace mm = mototron_msgs;
namespace mcm = marti_common_msgs;

namespace sumet_low_level_controller
{
Controller::Controller()
  :
  update_rate_hz_(25.0),
  require_brake_(false),
  robotic_mode_(true),
  engine_running_(true),
  ignition_on_(true)
{
}

Controller::~Controller()
{
}

void Controller::onInit()
{
  nh_ = getNodeHandle();
  pnh_ = getPrivateNodeHandle();
  
  // Create a one-shot timer to initialize everything after a brief
  // pause so that ROS has time to connect to rosout so that we
  // don't drop errors/info during initialization.
  double initialization_delay = 1.0;
  pnh_.param("initialization_delay_s", initialization_delay, 1.0);
  init_timer_ = nh_.createWallTimer(ros::WallDuration(initialization_delay),
                                    &Controller::initialize,
                                    this,
                                    true);
}

void Controller::initialize(const ros::WallTimerEvent &ignored)
{
  //add modules
  curvature_command_module_ = AddModule<CurvatureCommandModule>("curvature_command");
  gear_state_module_ = AddModule<GearStateModule>("gear_state");
  ignition_module_ = AddModule<IgnitionModule>("ignition");
  mototron_status_module_ = AddModule<MototronStatusModule>("mototron_status");
  odometry_module_ = AddModule<OdometryModule>("odometry");
  speed_command_module_ = AddModule<SpeedCommandModule>("speed_command");

  Reconfigure(); //add modules before calling

  //publishers
  speed_input_pub_ = nh_.advertise<mcm::Float32Stamped>(
    "speed_input", 3, false);
  
  curvature_input_pub_ = nh_.advertise<mcm::Float32Stamped>(
    "curvature_input", 3, false);

  dbw_state_pub_ =
    nh_.advertise<snm::DriveByWireState>("drive_by_wire_state", 3);

  //subscribers
  robotic_mode_sub_ = nh_.subscribe(
    "robotic_mode", 3,
    &Controller::handleRoboticMode, this);

  engine_running_sub_ = nh_.subscribe(
    "engine_running", 3,
    &Controller::handleEngineRunning, this);

  ignition_on_sub_ = nh_.subscribe(
    "ignition_on", 3,
    &Controller::handleIgnitionOn, this);
  
  //services
  reconfigure_srv_ = pnh_.advertiseService(
    "reconfigure",
    &Controller::ReconfigureService,
    this);

  //update timer
  update_timer_ = nh_.createTimer(ros::Duration(1.0/update_rate_hz_),
                                  &Controller::UpdateTimerCallback,
                                  this);
}

void Controller::Reconfigure()
{
  pnh_.param("update_rate_hz", update_rate_hz_, 25.0);
  ROS_INFO("llc: update_rate = %f", update_rate_hz_);
  // removed timer period update because it's not supported in ROS 2
  //update_timer_.setPeriod(ros::Duration(1.0/update_rate_hz_));

  for (size_t i = 0; i < modules_.size(); ++i)
    modules_[i]->Reconfigure();
}

void Controller::AddStop(const std::string &reason)
{
  stop_reasons_.push_back(reason);
}

void Controller::AddStopWithBrake(const std::string &reason)
{
  require_brake_ = true;
  stop_reasons_.push_back(reason);
}

void Controller::AddWarning(const std::string &warning)
{
  warnings_.push_back(warning);
}

bool Controller::VehicleStopped()
{
  if (std::abs(GetOdometry()->VehicleSpeed()) < 0.1 ||
      GetGearState()->inPark())
    return true;
  else
    return false;
}

void Controller::UpdateTimerCallback(const ros::TimerEvent& ev)
{
  Update();
}

void Controller::Update()
{
  ResetStopState();
  UpdateModules();

  //calculate and publish the speed input
  {
    speed_cmd_ = speed_command_module_->calculateSpeedCommand(StopMode());
    mcm::Float32StampedPtr out = boost::make_shared<mcm::Float32Stamped>();
    out->header.stamp = ros::Time::now();
    out->value = speed_cmd_;
    speed_input_pub_.publish(out);
  }

  //calculate and publish the curvature input
  {
    curvature_cmd_ = curvature_command_module_->calculateCurvatureCommand(speed_cmd_);
    mcm::Float32StampedPtr out = boost::make_shared<mcm::Float32Stamped>();
    out->header.stamp = ros::Time::now();
    out->value = curvature_cmd_;
    curvature_input_pub_.publish(out);
  }

  PostUpdateModules();
  PublishDbwState();
}

template <class ModuleType>
ModuleType* Controller::AddModule(const std::string &name)
{
  ModuleType* module = new ModuleType();
  module->InitializeModule(this, name, nh_, pnh_);
  modules_.push_back(module);
  return module;
}

void Controller::DeleteModules()
{
  for (size_t i = 0; i < modules_.size(); ++i)
  {
    modules_[i]->ShutdownModule();
    delete modules_[i];
    modules_[i] = NULL;
  }
  modules_.clear();
}

void Controller::UpdateModules()
{
  for (size_t i = 0; i < modules_.size(); ++i)
    modules_[i]->Update();
}

void Controller::PostUpdateModules()
{
  for (size_t i = 0; i < modules_.size(); ++i)
    modules_[i]->PostUpdate();
}

void Controller::ResetStopState()
{
  require_brake_ = false;
  stop_reasons_.clear();
  warnings_.clear();
}

bool Controller::StopMode() const
{
  if (stop_reasons_.empty())
    return false;
  else
    return true;
}

void Controller::PublishDbwState()
{
  snm::DriveByWireStatePtr msg =
    boost::make_shared<snm::DriveByWireState>();
  
  msg->header.stamp = ros::Time::now();
//  msg->header.frame_id = std::string();

  msg->gear_mode = GetGearState()->gearMode();
  msg->desired_gear = GetGearState()->desiredGear();
  msg->gear = GetGearState()->currentGear();

  //TODO, remove invalid members from the msg
  const double invalid = std::numeric_limits<double>::quiet_NaN();
  msg->curvature_command = curvature_cmd_;
  msg->curvature_measure = invalid;
  msg->steering_percent_command = invalid;
  msg->steering_percent_measure = invalid;

  msg->gasbrake_percent_command = invalid;
  msg->gasbrake_percent_measure = invalid;

  msg->speed_command = speed_cmd_;

  msg->ito_enabled = !roboticMode();
  msg->vehicle_stopped = VehicleStopped();

  msg->ignition_on = ignitionOn();
  msg->engine_running = engineRunning();

  msg->high_idle_state = false;
  msg->awd_active = false;

  msg->estop_sense = GetMototronStatus()->EStopSense();

  msg->dbw_stop_active = StopMode();
  msg->dbw_stop_reasons = stop_reasons_;
  msg->dbw_warnings = warnings_;

  dbw_state_pub_.publish(msg);
}

bool Controller::ReconfigureService(std_srvs::Empty::Request &request,
                                    std_srvs::Empty::Response &response)
{
  Reconfigure();
  return true;
}

void Controller::handleRoboticMode(const mcm::BoolStampedConstPtr &msg)
{
  robotic_mode_ = msg->value;
}

void Controller::handleEngineRunning(const mcm::BoolStampedConstPtr &msg)
{
  engine_running_ = msg->value;
}

void Controller::handleIgnitionOn(const mcm::BoolStampedConstPtr &msg)
{
  ignition_on_ = msg->value;
}

}  // namespace sumet_low_level_controller

// Register nodelet plugin
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(
    sumet_low_level_controller::Controller,
    nodelet::Nodelet)
