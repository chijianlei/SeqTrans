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

#include <sumet_low_level_controller/mototron_status_module.h>
#include <ros/ros.h>

namespace mm = mototron_msgs;

namespace sumet_low_level_controller
{
MototronStatusModule::MototronStatusModule()
  :
  timeout_s_(-1.0),
  status_received_(false),
  status_valid_(false),
  status_error_("<UNINITIALIZED>")
{
}

MototronStatusModule::~MototronStatusModule()
{
}

void MototronStatusModule::Reconfigure()
{
  ros::NodeHandle mnh = GetModuleNodeHandle();

  mnh.param("timeout_s", timeout_s_, -1.0);
  ROS_INFO("%s: timeout_s = %f", GetName().c_str(), timeout_s_);
}

void MototronStatusModule::Update()
{
  UpdateStatusValid();

  if (!status_valid_)
    AddStop(status_error_);
}

void MototronStatusModule::PostUpdate()
{
  // After the control iteration has finished, we add
  // inactive/stop/estop states based on the mototron status.  By
  // adding them after the iteration, they do not affect the command
  // sent to the Mototron so we avoid any hysteretic problems, but
  // they will still be published in the drive by wire state so that
  // external users are aware of why the vehicle is stopping.
  if (!status_valid_)
    return;

  if (status_msg_.ito_active)
    AddWarning("Mototron has detected ITO active.");
}

void MototronStatusModule::HandleMototronStatus(
  const mm::MototronStatusConstPtr &msg)
{
  status_received_ = true;
  status_msg_ = *msg;
  UpdateStatusValid();
}

bool MototronStatusModule::Valid() const
{
  return status_valid_;
}

bool MototronStatusModule::VehicleStopped() const
{
  return status_msg_.vehicle_stopped;
}

bool MototronStatusModule::EStopSense() const
{
  return status_msg_.estop_sense;
}

void MototronStatusModule::Initialize()
{
  Reconfigure();

  ros::NodeHandle nh = GetNodeHandle();
  status_sub_ = nh.subscribe("/can/mototron_status", 3,
                             &MototronStatusModule::HandleMototronStatus,
                             this);

  status_received_ = false;
  UpdateStatusValid();
}

void MototronStatusModule::Shutdown()
{
  status_sub_.shutdown();
}

void MototronStatusModule::UpdateStatusValid()
{
  double age = (ros::Time::now() - status_msg_.header.stamp).toSec();
  if (timeout_s_ > 0 && age > timeout_s_)
  {
    char buffer[1024];
    snprintf(buffer, sizeof(buffer),
             "Mototron status has timed out (%f > %f)",
             age, timeout_s_);

    status_valid_ = false;
    status_error_ = buffer;
    return;
  }

  status_valid_ = true;
  status_error_ = "no error";
}
}  // namespace sumet_low_level_controller
