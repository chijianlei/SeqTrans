#include <speed_controller/longitudinal_dynamics.h>
#include <speed_controller/rootfinding_util.h>

#include "rclcpp/rclcpp.hpp"

namespace speed_controller
{

void LongitudinalDynamicsModel::getParams(rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parent_interface)
{
  delay = (parent_interface->get_parameter("delay")).as_double();
  throttle_deadband = (parent_interface->get_parameter("throttle_deadband")).as_double();
  throttle_gain = (parent_interface->get_parameter("throttle_gain")).as_double();
  idle_power = (parent_interface->get_parameter("idle_power")).as_double();
  speed_offset = (parent_interface->get_parameter("speed_offset")).as_double();
  peak_force = (parent_interface->get_parameter("peak_force")).as_double(); //set < 0 to disable
  brake_deadband = (parent_interface->get_parameter("brake_deadband")).as_double();
  brake_gain = (parent_interface->get_parameter("brake_gain")).as_double();
  brake_gain2 = (parent_interface->get_parameter("brake_gain2")).as_double();
  resistance_coeff_1 = (parent_interface->get_parameter("resistance_coeff_1")).as_double();
  resistance_coeff_v = (parent_interface->get_parameter("resistance_coeff_v")).as_double();
  resistance_coeff_v2 = (parent_interface->get_parameter("resistance_coeff_v2")).as_double();
  grav_gain = (parent_interface->get_parameter("grav_gain")).as_double();
}

inline double sign(double val)
{
  return (val > 0.0) - (val < 0.0);
}

bool LongitudinalDynamicsModel::calcAcceleration(
    const double throttle,
    const double brake,
    const double velocity,
    const double ground_pitch,
    const bool in_reverse,
    double& acceleration) const
{
  //propulsive forces
  double speed = velocity * (1.0 - in_reverse*2.0);
  speed = std::max(speed, 0.0);
  double throttle_adj = std::max(throttle - throttle_deadband, 0.0); //adjust for deadband
  double Pe = throttle_gain*throttle_adj + idle_power; //engine power
  double Fe = Pe/(speed + speed_offset); //engine force
  if (peak_force >= 0.0)
  {
    Fe = std::min(Fe, peak_force);
  }
  Fe = Fe * (1.0 - in_reverse*2.0);

  double Fg = grav_gain*grav_acc*sin(ground_pitch); //gravitational force
  double Fp = Fe + Fg;

  //dissipative forces

  double brake_adj = std::max(brake - brake_deadband, 0.0); //adjust for deadband
  double abs_Fb = brake_gain*brake_adj + brake_gain2*(brake_adj*brake_adj); //brake force
  double abs_Fr = (std::abs(velocity) > 0.0 ? resistance_coeff_1 : 0.0) +
      resistance_coeff_v*std::abs(velocity) +
      resistance_coeff_v2*(velocity*velocity); //resistance force

  double Fd;
  if (std::abs(velocity) > 0.0)
  {
    //the vehicle is moving. Fb and Fr act to dissipate energy (i.e. slow the vehicle down)
    Fd = -sign(velocity)*(abs_Fb + abs_Fr);
  }
  else
  {
    //the vehicle is stopped. Fb and Fr act to prevent motion
    Fd = -sign(Fp)*(abs_Fb + abs_Fr);
    if (abs(Fd) > abs(Fp))
    {
      Fd = -Fp; //can't exceed propulsive forces
    }
  }

  //forces are all specific forces (normalized by mass)
  acceleration = Fp + Fd;

  RCLCPP_DEBUG(logger_, "  throttle = %lf, brake = %lf, velocity = %lf, ground_pitch = %lf, in_reverse = %d, "
           "Fe = %lf, Fg = %lf, abs_Fb = %lf, abs_Fr = %lf, Fp = %lf, Fd = %lf, accel = %lf",
           throttle, brake, velocity, ground_pitch, in_reverse,
           Fe, Fg, abs_Fb, abs_Fr, Fp, Fd, acceleration); //DEBUGGING

  return true;
}

bool LongitudinalDynamicsModel::calcThrottleBrake(
    const double acceleration,
    const double velocity,
    const double ground_pitch,
    const bool in_reverse,
    double& throttle_brake) const
{
  //inverse of the calcAccel function
  double tolx = 1e-3;
  double tolfun = 1e-3;
  int maxiter = 50;

  double target_accel;

  //lambda error function
  //must capture target_accel by reference because value may change
  //after the lambda declaration
  auto error_function = [this, velocity, ground_pitch, in_reverse, &target_accel](double tb)->double
  {
    double throttle = std::max(tb, 0.0);
    double brake = std::max(-tb, 0.0);
    double pred_accel;
    this->calcAcceleration(throttle,brake,velocity,ground_pitch,in_reverse,pred_accel);
    double error = pred_accel - target_accel;

    RCLCPP_DEBUG(this->logger_, "  gasbrake = %lf, target_accel = %lf, pred_accel = %lf", tb, target_accel, pred_accel); //DEBUGGING

    return error;
  };

  int exitflag = 0;
  if (velocity == 0.0 && acceleration == 0.0)
  {
      //find the minimum required brake force to remain stopped
      double no_brake_accel;
      this->calcAcceleration(0.0,0.0,velocity,ground_pitch,in_reverse,no_brake_accel);
      if (std::abs(no_brake_accel) > 0.0)
      {
        target_accel = sign(no_brake_accel)*.001; //acceleration epsilon
        exitflag = findRootBrents(error_function, -1.0, 0.0, tolx, tolfun, maxiter, throttle_brake);
      }
      else
      {
        throttle_brake = 0.0;
        exitflag = FindRootBrentsFlags::SUCCESS;
      }
      //TODO, add a margin to brake command when stopped (outside of this function)
  }
  else
  {
    target_accel = acceleration;
//    ROS_INFO("Find root in gasbrake range [-1.0, 0.0]"); //DEBUGGING
    exitflag = findRootBrents(error_function, -1.0, 0.0, tolx, tolfun, maxiter, throttle_brake); //first try brake
    if (exitflag < 0)
    {
//      ROS_INFO("Find root in gasbrake range [0.0, 1.0]"); //DEBUGGING
      exitflag = findRootBrents(error_function, 0.0, 1.0, tolx, tolfun, maxiter, throttle_brake); //then try throttle
    }
  }

  if (exitflag < 0)
  {
    RCLCPP_WARN(logger_, "Failed to find root, exitflag = %d", exitflag);
    throttle_brake = 0.0;
    return false;
  }
  return true;
}

} //namespace
