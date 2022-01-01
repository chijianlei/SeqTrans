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
#include <sumet_low_level_controller/ignition_module.h>
#include <sumet_low_level_controller/gear_state_module.h>

namespace mcm = marti_common_msgs::msg;
namespace snm = sumet_nav_msgs::srv;

namespace sumet_low_level_controller
{
IgnitionModule::IgnitionModule()
  :
  start_engine_time_s_(-1.0),
  starting_engine_(false)
{
}

IgnitionModule::~IgnitionModule()
{
}

void IgnitionModule::Reconfigure()
{
  auto mnh = GetModuleNodeHandle();

  start_engine_time_s_ = (get_sub_parameter(mnh, "start_engine_time_s")).as_double();
  RCLCPP_INFO(mnh->get_logger(), "%s: start_engine_time_s = %f", GetName().c_str(), start_engine_time_s_);
}

void IgnitionModule::Update()
{
  auto mnh = GetModuleNodeHandle();
  if (!starting_engine_)
    return;

  double age = (mnh->now() - start_request_time_).seconds();
  if (start_engine_time_s_ > 0 && age < start_engine_time_s_)
  {
    char buffer[1024];
    snprintf(buffer, sizeof(buffer),
             "Waiting for engine start to finish. (%f s remaining)",
             start_engine_time_s_ - age);
    AddStop(buffer);
    return;
  }
  else
    starting_engine_ = false;
}

void IgnitionModule::Initialize()
{
  auto nh = GetNodeHandle();
  start_engine_srv_ = nh->create_service<sumet_nav_msgs::srv::StartEngine>(
    "start_engine",
    std::bind(&IgnitionModule::StartEngineService,
      this,
      std::placeholders::_1,
      std::placeholders::_2,
      std::placeholders::_3
    )
  );

  stop_engine_srv_ = nh->create_service<sumet_nav_msgs::srv::StopEngine>(
    "stop_engine",
    std::bind(&IgnitionModule::StopEngineService,
      this,
      std::placeholders::_1,
      std::placeholders::_2,
      std::placeholders::_3
    )
  );

  run_engine_pub_ = nh->create_publisher<mcm::BoolStamped>(
    "run_engine", rclcpp::QoS(1));

  auto mnh = GetModuleNodeHandle();
  declare_sub_parameter(mnh, "start_engine_time_s", rclcpp::ParameterValue(-1.0));

  Reconfigure();
}

void IgnitionModule::Shutdown()
{
}

bool IgnitionModule::StartEngineService(const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<sumet_nav_msgs::srv::StartEngine::Request> request,
  const std::shared_ptr<sumet_nav_msgs::srv::StartEngine::Response> response)
{
  auto mnh = GetModuleNodeHandle();
  RCLCPP_INFO(mnh->get_logger(),
    "%s: StartEngineService called.",
    GetName().c_str()
  );
  response->result = snm::StartEngine_Response::SUCCESS;

  if (!GetController()->roboticMode())
    response->result |= snm::StartEngine_Response::ERROR_ITO_ACTIVE;
  if (GetController()->engineRunning())
    response->result |= snm::StartEngine_Response::ERROR_ALREADY_RUNNING;
  if (!GetController()->ignitionOn())
    response->result |= snm::StartEngine_Response::ERROR_IGNITION_OFF;

  if (!GetController()->GetGearState()->inPark())
  {
    response->result |= snm::StartEngine_Response::ERROR_NOT_IN_PARK;
  }

  if (response->result == snm::StartEngine_Response::SUCCESS) {
    RCLCPP_INFO(mnh->get_logger(),
      "%s: requesting engine start.",
      GetName().c_str()
    );
    sendRunMessage(true);
    starting_engine_ = true;
    start_request_time_ = mnh->now();
  }

  return true;
}

bool IgnitionModule::StopEngineService(const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<sumet_nav_msgs::srv::StopEngine::Request> request,
  const std::shared_ptr<sumet_nav_msgs::srv::StopEngine::Response> response)
{
  auto mnh = GetModuleNodeHandle();
  RCLCPP_INFO(mnh->get_logger(),
    "%s: StopEngineService called.",
    GetName().c_str()
  );
  response->result = snm::StopEngine_Response::SUCCESS;

  if (!GetController()->roboticMode())
    response->result |= snm::StopEngine_Response::ERROR_ITO_ACTIVE;
  if (!GetController()->engineRunning())
    response->result |= snm::StopEngine_Response::ERROR_ALREADY_STOPPED;

  if (response->result == snm::StopEngine_Response::SUCCESS) {
    RCLCPP_INFO(mnh->get_logger(),
      "%s: requesting engine stop.",
      GetName().c_str()
    );
    sendRunMessage(false);
  }

  return true;
}

void IgnitionModule::sendRunMessage(const bool run)
{
  auto mnh = GetModuleNodeHandle();
  mcm::BoolStamped::SharedPtr msg = std::make_shared<mcm::BoolStamped>();
  msg->header.stamp = mnh->now();
  msg->value = run;
  run_engine_pub_->publish(*msg);
}
}  // namespace sumet_low_level_controller
