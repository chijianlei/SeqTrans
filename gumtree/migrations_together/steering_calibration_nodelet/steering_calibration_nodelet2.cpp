// *****************************************************************************
//
// Copyright (C) 2017 All Right Reserved, Southwest Research Institute® (SwRI®)
//
// THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
// KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
// PARTICULAR PURPOSE.
//
// *****************************************************************************

/** @file ROS2 implementation of the steering_calibration_nodelet for the RTK
 * motion executor
 *
 * @detail Note that this is not actually a nodelet, which isn't used in ROS2.
 * We've retained the same name as the ROS1 code for consistency.
 */

#include <thread>

#include "rclcpp/rclcpp.hpp"
#include <diagnostic_updater/diagnostic_updater.hpp>

#include <marti_common_msgs/msg/float32_stamped.hpp>
#include <rtk_steering_calibration/ackermann.h>

// Alias type for easier access to DiagnosticStatus enumerations.
typedef diagnostic_msgs::msg::DiagnosticStatus DS;
namespace du = diagnostic_updater;
namespace mcm = marti_common_msgs::msg;

namespace rtk_steering_calibration
{
  class SteeringCalibrationNodelet : public rclcpp::Node
  {
    std::shared_ptr<du::Updater> diagnostic_updater_;
    rclcpp::TimerBase::SharedPtr diagnostic_timer_;

    rclcpp::TimerBase::SharedPtr init_timer_;

    rclcpp::Subscription<mcm::Float32Stamped>::SharedPtr steering_sense_sub_;
    rclcpp::Subscription<mcm::Float32Stamped>::SharedPtr curvature_setpoint_sub_;

    rclcpp::Publisher<mcm::Float32Stamped>::SharedPtr steering_setpoint_pub_;
    rclcpp::Publisher<mcm::Float32Stamped>::SharedPtr curvature_sense_pub_;

    AckermannParameters cal_;

    void onInit()
    {
      // create a one-shot timer to initialize everything after a brief
      // pause so that ros has time to connect to /rosout and we don't
      // drop errors/info during initialization.
      double initialization_delay = 1.0;
      std::this_thread::sleep_for(
        std::chrono::duration<int, std::milli>(int(1000*initialization_delay))
      );
      this->initialize();
    }
    void initialize()
    {
      RCLCPP_INFO(this->get_logger(),"initialize()");
      // read the initial configuration off the parameter server
      cal_.zero_curvature_steering = (this->get_parameter("zero_curvature_steering")).as_double();
      cal_.center_slope = (this->get_parameter("center_slope")).as_double();
      cal_.distortion_factor = (this->get_parameter("distortion_factor")).as_double();

      // Steering -> Curvature conversion support
      steering_sense_sub_ = this->create_subscription<mcm::Float32Stamped>("steering_sense",2,
        std::bind(&SteeringCalibrationNodelet::handleSteeringSense,this,std::placeholders::_1));
      curvature_sense_pub_ = this->create_publisher<mcm::Float32Stamped>("curvature_sense", rclcpp::QoS(2));

      // Curvature -> Steering conversion support
      curvature_setpoint_sub_ = this->create_subscription<mcm::Float32Stamped>(
        "curvature_setpoint", 2,
        std::bind(&SteeringCalibrationNodelet::handleCurvatureSetpoint,this,std::placeholders::_1));
      steering_setpoint_pub_ = this->create_publisher<mcm::Float32Stamped>("steering_setpoint", rclcpp::QoS(2));

      // Diagnostics
      diagnostic_timer_ = this->create_wall_timer(std::chrono::duration<int, std::milli>(1000),
        std::bind(&SteeringCalibrationNodelet::handleDiagnosticTimer,this));
      diagnostic_updater_ = std::make_shared<du::Updater>(this->create_sub_node("rtk_steering_calibration_diagnostics"));
      diagnostic_updater_->setHardwareID("none");
      diagnostic_updater_->add(
        "Steering Calibration", this,
        &SteeringCalibrationNodelet::updateDiagnostics);
      return;
    }

    void handleDiagnosticTimer()
    {
      diagnostic_updater_->update();
    }

    void handleSteeringSense(const mcm::Float32Stamped::SharedPtr in_msg)
    {
      auto out_msg = mcm::Float32Stamped();
      out_msg.header.stamp = in_msg->header.stamp;
      curvaturesFromSteerings(&out_msg.value, &cal_, &in_msg->value, 1);
      curvature_sense_pub_->publish(out_msg);
    }

    void handleCurvatureSetpoint(const mcm::Float32Stamped::SharedPtr in_msg)
    {
      auto out_msg = mcm::Float32Stamped();
      out_msg.header.stamp = in_msg->header.stamp;
      steeringsFromCurvatures(&out_msg.value, &cal_, &in_msg->value, 1);
      steering_setpoint_pub_->publish(out_msg);
    }

    void updateDiagnostics(du::DiagnosticStatusWrapper& status)
    {
      status.summary(DS::OK, "No errors reported.");
    }

  public:
    SteeringCalibrationNodelet(rclcpp::NodeOptions options)
    : Node("steering_calibration_nodelet", options)
    {
      // declare parameters and defaults
      this->declare_parameter("zero_curvature_steering",
        rclcpp::ParameterValue(0.5f));
      this->declare_parameter("center_slope",
        rclcpp::ParameterValue(0.2f));
      this->declare_parameter("distortion_factor",
        rclcpp::ParameterValue(4.0f));
      // wrap ROS1 nodelet initializer
      onInit();
    }
  }; // class SteeringCalibrationNodelet
} // namespace rtk_steering_calibration

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(rtk_steering_calibration::SteeringCalibrationNodelet)
