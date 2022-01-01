#include <string>
#include <sstream>

#include <ros/ros.h>
#include <speed_controller/simple_pid.h>

namespace speed_controller
{

  marti_common_msgs::Float32StampedPtr makeFloat32StampedPtr(
      ros::Time stamp,
      double value)
  {
    marti_common_msgs::Float32StampedPtr msg = boost::make_shared<marti_common_msgs::Float32Stamped>();
    msg->header.stamp = stamp;
    msg->value = value;
    return msg;
  }

  ControlMapConfig::ControlMapConfig():
    min_brake_(0.0),
    max_brake_(1.0),
    brake_stop_(0.6),
    min_throttle_(0.0),
    max_throttle_(1.0),
    min_stop_brake_(0.0)
  {
  }

  void ControlMapConfig::SetMap(double min_brake, double max_brake, double brake_stop,
                                double min_throttle, double max_throttle, double min_stop_brake)
  {
    // If min_brake is greater than max_brake, force min_brake to be equal to max brake
    if (min_brake > max_brake)
    {
      ROS_WARN("min_brake (%f) cannot be greater than max_brake (%f); changing min_brake to %f",
          min_brake, max_brake, min_brake);
      min_brake = max_brake;
    }
    // same for throttle
    if (min_throttle > max_throttle)
    {
      ROS_WARN("min_throttle (%f) cannot be greater than max_throttle (%f); changing min_throttle to %f",
          min_throttle, max_throttle, min_throttle);
      min_throttle = max_throttle;
    }

    min_brake_ = min_brake;
    max_brake_ = max_brake;
    brake_stop_ = brake_stop;
    min_throttle_ = min_throttle;
    max_throttle_ = max_throttle;
    min_stop_brake_ = min_stop_brake;
  }

  double ControlMapConfig::MapThrottle(double value)
  {
    // Map the throttle range (0,1]-> (min_throttle_, max_throttle_]
    // leaving zero values unmapped
    if (value > 0.0)
    {
      double range = max_throttle_ - min_throttle_;
      value = value * range + min_throttle_;
    }
    else
    {
      // clamp all other values to zero
      value = 0.0;
    }
    return value;
  }

  double ControlMapConfig::MapBrake(double value, bool cmd_stop)
  {
    // Map the brake range (0,1]-> (min_brake_, max_brake_]
    // leaving zero values unmapped
    if (value > 0.0)
    {
      double range = max_brake_ - min_brake_;
      value = value * range + min_brake_;

      // Testing a minimum brake when stopping
      if (cmd_stop)
      {
        value = std::max(value, min_stop_brake_);
      }
    }
    else
    {
      // clamp all other values to zero
      value = 0.0;
    }

    return value;
  }

  void ControlMapConfig::PrintConfig()
  {
    ROS_INFO("Control Map Configuration: \n"
             "min_brake = %lf\n"
             "max_brake = %lf\n"
             "min_throttle = %lf\n"
             "max_throttle = %lf\n"
             "stop_brake = %lf"
             "min_stop_brake = %lf\n",
             min_brake_, max_brake_,
             min_throttle_, max_throttle_,
             brake_stop_, min_stop_brake_);
  }

  PidConfig::PidConfig():
    kp_(0.0),
    ki_(0.0),
    kd_(0.0),
    kp_stop_(0.0),
    ki_stop_(0.0),
    kd_stop_(0.0),
    min_i_(0.0),
    max_i_(0.0),
    min_d_(0.0),
    max_d_(0.0),
    feedforward_vs_speed_ratio_(0.0),
    measured_filter_cut_off_hz_(0.0),
    derr_filter_cut_off_hz_(0.0)
  {
  }

  void PidConfig::SetGains(double kp, double ki, double kd)
  {
    if (kp < 0.0 || ki < 0.0 || kd < 0.0)
    {
      ROS_WARN("Control gains must be >= 0. Requested: kp = %lf, ki = %lf, kd = %lf",
          kp, ki, kd);
    }

    kp_ = std::max(kp,0.0);
    ki_ = std::max(ki,0.0);
    kd_ = std::max(kd,0.0);

    //check that (kx + kx_stop) still >= 0
    SetGainsStop(kp_stop_, ki_stop_, kd_stop_);
  }

  void PidConfig::SetGainsStop(double kp_stop, double ki_stop, double kd_stop)
  {
    if (kp_stop < -kp_ || ki_stop < -ki_ || kd_stop < -kd_)
    {
      ROS_WARN("Sum of (kx + kx_stop) must be >= 0. Requested: kp_stop = %lf, ki_stop = %lf, kd_stop = %lf",
          kp_stop, ki_stop, kd_stop);
    }

    kp_stop_ = std::max(kp_stop, -kp_);
    ki_stop_ = std::max(ki_stop, -ki_);
    kd_stop_ = std::max(kd_stop, -kd_);
  }

  void PidConfig::SetLimits(double min_i, double max_i,
                            double min_d, double max_d)
  {
    if (min_i > 0 || min_d > 0)
    {
      ROS_WARN("min_x values can't be > 0. Requested min_i = %f, min_d = %f", min_i, min_d);
    }
    if (max_i < 0 || max_d < 0)
    {
      ROS_WARN("max_x values can't be < 0. Requested max_i = %f, max_d = %f", max_i, max_d);
    }

    min_i_ = std::min(min_i,0.0);
    max_i_ = std::max(max_i,0.0);
    min_d_ = std::min(min_d,0.0);
    max_d_ = std::max(max_d,0.0);
  }

  void PidConfig::SetFeedforwardVsSpeedRatio(double value)
  {
    feedforward_vs_speed_ratio_ = value;
  }

  double infinity() { return std::numeric_limits<double>::infinity(); }

  void PidConfig::SetFilterCutOffHz(
      double measured,
      double derr)
  {
    measured_filter_cut_off_hz_ = measured;
    derr_filter_cut_off_hz_ = derr;
  }

  void PidConfig::PrintConfig()
  {
    ROS_INFO("PID Configuration: \n"
             "kp = %lf\n"
             "ki = %lf\n"
             "kd = %lf\n"
             "kp_stop = %lf\n"
             "ki_stop = %lf\n"
             "kd_stop = %lf\n"
             "min_i = %lf\n"
             "max_i = %lf\n"
             "min_d = %lf\n"
             "max_d = %lf\n"
             "feedforward_vs_speed_ratio = %lf\n"
             "measured_filter_cut_off_hz = %lf\n"
             "derr_filter_cut_off_hz = %lf",
             kp_, ki_, kd_,
             kp_stop_, ki_stop_, kd_stop_,
             min_i_, max_i_,
             min_d_, max_d_,
             feedforward_vs_speed_ratio_,
             measured_filter_cut_off_hz_,
             derr_filter_cut_off_hz_);
  }

  SimplePid::SimplePid():

    last_measured_(0.0),
    last_commanded_(0.0),
    initialized_(false),
    last_err_(0.0),
    last_integral_err_(0.0),
    measured_filtered_(0.0),
    derr_filtered_(0.0),
    last_update_time_(ros::Time(0))
  {
  }

  void SimplePid::Reset()
  {
    initialized_ = false;
  }

  inline double calcAlphaEWMA(double cut_off_hz, double dt)
  {
    if (std::isinf(cut_off_hz) || cut_off_hz <= 0.0 || dt <= 0.0)
    {
      return 0.0; //invalid input. return 0.0 for no filter
    }

    //calculate alpha for exponential weighted moving average given
    //cut off frequency and time step
    //http://blog.prosig.com/2003/04/28/data-smoothing-rc-filtering-and-exponential-averaging/
    double time_constant = 1/(2.0*M_PI*cut_off_hz);
    double N = time_constant/dt + 1;
    double alpha = (N-1)/N;
    return alpha;
  }

  double SimplePid::Update(const ros::Time& time)
  {
    //input the current time vs. using ros::Time::now()
    //so that Update can be run faster than real time
    //in simulation.

    double control_out = 0.0;

    if (!initialized_)
    {
      last_err_ = 0.0;
      last_integral_err_ = 0.0;
      measured_filtered_ = 0.0;
      derr_filtered_ = 0.0;
      initialized_ = true;
    }
    else
    {
      //set control gains
      double kp = config_.GetKp();
      double ki = config_.GetKi();
      double kd = config_.GetKd();
      if (last_commanded_ == 0.0)
      {
        //apply additive gains when commanding a stop
        kp += config_.GetKpStop(); //to respond more aggressively when stopping for obstacle
        ki += config_.GetKiStop(); //to prevent creeping or rolling downhill
        kd += config_.GetKdStop();
      }

      double dt = (time - last_update_time_).toSec();

      //filter measured value
      double alpha = calcAlphaEWMA(config_.GetMeasuredFilterCutOffHz(), dt);
      measured_filtered_ = alpha*measured_filtered_ + (1.0 - alpha)*last_measured_;

      //compute proportional contribution
      double err = last_commanded_ - measured_filtered_;
      double p = kp*err;
      //compute integral contribution
      double integral_err = 0.0; //keep integral err at zero when ki is zero
      if (ki > 0.0)
      {
        //clamp integral error to mitigate windup
        integral_err = ClampIntegralErr(last_integral_err_ + err*dt, ki);
      }
      double i = ki*integral_err;
      //compute derivative contribution
      if (dt > 0.0) //check to prevent divide by zero
      {
        double derr = (err - last_err_) / dt;
        //filter the derr value
        double alpha = calcAlphaEWMA(config_.GetDerrFilterCutOffHz(), dt);
        derr_filtered_ = alpha*derr_filtered_ + (1.0 - alpha)*derr;
      } //else leave derr_filtered_ unchanged
      double d = ClampD(kd*derr_filtered_);
      //compute feedforward contribution
      double ff = config_.GetFeedforwardVsSpeedRatio() * last_commanded_;
     
      double raw_control_out = ff + p + i + d;
      control_out = ClampControl(raw_control_out);
      
      //set for next call to update
      last_err_ = err;
      last_integral_err_ = integral_err;
    }

    last_update_time_ = time;
    return control_out;
  }


  void SimplePid::LoadMeasured(double value)
  {
    last_measured_ = value;
  }

  void SimplePid::LoadCommanded(double value)
  {
    last_commanded_ = value;
  }

  void SimplePid::SetConfig(const PidConfig& config)
  {
    config_ = config;
//    Reset(); //TODO, should reset integral error when config changes?
  }

  double SimplePid::ClampIntegralErr(double val, double ki)
  {
    if (ki > 0.0) //check to prevent divide by zero
    {
      double max_integral_err = config_.GetIntegralMax()/ki;
      double min_integral_err = config_.GetIntegralMin()/ki;
      if (val > max_integral_err)
      {
        val = max_integral_err;
      }
      else if(val < min_integral_err)
      {
        val = min_integral_err;
      }
    }
    return val;
  }

  double SimplePid::ClampD(double val)
  {
    if (val > config_.GetDerivativeMax())
    {
      val = config_.GetDerivativeMax();
    }
    else if(val < config_.GetDerivativeMin())
    {
      val = config_.GetDerivativeMin();
    }
    return val;
  }

  double SimplePid::ClampControl(double val)
  {
    if (val > 1.0)
    {
      val = 1.0;
    }
    else if(val < -1.0)
    {
      val = -1.0;
    }
    return val;
  }
}
