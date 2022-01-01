#include <sumet_low_level_controller/curvature_command_module.h>
#include <sumet_low_level_controller/speed_command_module.h>
#include <sumet_low_level_controller/odometry_module.h>
#include <sumet_low_level_controller/gear_state_module.h>

namespace sumet_low_level_controller
{

CurvatureCommandModule::CurvatureCommandModule()
:
timeout_s_(-1.0),
curvature_valid_(false),
curvature_error_("<UNINITIALIZED>"),
last_cmd_curvature_(0.0)
{
}

CurvatureCommandModule::~CurvatureCommandModule()
{
}

void CurvatureCommandModule::Reconfigure()
{
  auto mnh = GetModuleNodeHandle();

  timeout_s_ = (get_sub_parameter(mnh, "timeout_s")).as_double();
  RCLCPP_INFO(mnh->get_logger(),
    "%s: timeout_s = %f", GetName().c_str(), timeout_s_);

  //recheck if valid
  curvature_valid_ = false;
  UpdateCurvatureValid();
}

void CurvatureCommandModule::Update()
{
  UpdateCurvatureValid();

  if (!curvature_valid_)
  {
    AddStop(curvature_error_);
  }
}

void CurvatureCommandModule::Initialize()
{
  auto nh = GetNodeHandle();
  curvature_sub_ = nh->create_subscription<marti_common_msgs::msg::Float32Stamped>(
    "curvature_setpoint", 2,
    std::bind(&CurvatureCommandModule::handleCurvatureMsg, this,std::placeholders::_1)
  );
  auto mnh = GetModuleNodeHandle();
  declare_sub_parameter(mnh, "timeout_s", rclcpp::ParameterValue(timeout_s_));
  Reconfigure();
}

void CurvatureCommandModule::Shutdown()
{
  curvature_sub_.reset(); // HACKY ros2 analog
}

void CurvatureCommandModule::UpdateCurvatureValid()
{
  /// Removed safety checks based on swri_roscpp, this is a stub now

  curvature_valid_ = true;
  curvature_error_ = "no error";
}

double CurvatureCommandModule::calculateCurvatureCommand(
    const double speed_command)
{
  double cmd_curvature;

  bool stopped = GetController()->GetOdometry()->VehicleStopState();
  int32_t desired_gear = GetController()->GetGearState()->desiredGear();
  int32_t current_gear = GetController()->GetGearState()->currentGear();

  if (speed_command == 0.0 && stopped) //commanded and actual speed are zero
  {
    cmd_curvature = last_cmd_curvature_;
//    ROS_WARN_THROTTLE(1.0, "Keep commanded curvature at %f because stopped.", cmd_curvature);
  }
  else if (current_gear != desired_gear)
  {
    cmd_curvature = last_cmd_curvature_;
//    ROS_WARN_THROTTLE(1.0, "Keep commanded curvature at %f because waiting for gear change (%d -> %d).",
//        cmd_curvature, current_gear, desired_gear);
  }
  else if(curvature_valid_)
  {
    cmd_curvature = curvature_msg_->value;
  }
  else
  {
    cmd_curvature = 0.0;
    auto nh = GetNodeHandle();
    RCLCPP_WARN(nh->get_logger(), "Curvature invalid!");
  }
  last_cmd_curvature_ = cmd_curvature;

  return cmd_curvature;
}

void CurvatureCommandModule::handleCurvatureMsg(const marti_common_msgs::msg::Float32Stamped::SharedPtr msg)
{
    curvature_msg_ = msg;
}

}
