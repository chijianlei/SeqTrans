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
  ros::NodeHandle mnh = GetModuleNodeHandle();

  mnh.param("timeout_s", timeout_s_, timeout_s_);
  ROS_INFO("%s: timeout_s = %f", GetName().c_str(), timeout_s_);

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
  ros::NodeHandle nh = GetNodeHandle();
  curvature_sub_ = nh.subscribe("curvature_setpoint", 2, &CurvatureCommandModule::handleCurvatureMsg, this);
  Reconfigure();
}

void CurvatureCommandModule::Shutdown()
{
  curvature_sub_ = ros::Subscriber();
}

void CurvatureCommandModule::UpdateCurvatureValid()
{
  /// Removed safety checks, this is a stub now

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
    ROS_WARN_THROTTLE(1.0, "Curvature invalid!");
  }
  last_cmd_curvature_ = cmd_curvature;

  return cmd_curvature;
}

void CurvatureCommandModule::handleCurvatureMsg(const marti_common_msgs::Float32StampedConstPtr& msg)
{
    curvature_msg_ = msg;
}

}
