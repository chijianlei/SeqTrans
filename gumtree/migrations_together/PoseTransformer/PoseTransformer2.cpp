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

#include <sumet_state_estimator/PoseTransformer.h>
#include <limits>


#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <swri_roscpp/logging.h>

namespace sumet_state_estimator
{
  PoseTransformer::PoseTransformer()
  {
    PoseTransformer("");
  }

  PoseTransformer::PoseTransformer(const std::string& world_frame)
  {
    world_frame_ = std::string(world_frame);
  }

  void PoseTransformer::initialize(std::shared_ptr<rclcpp::Node> node)
  {
    handle_ = node;
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>();
    listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, true);//handle_, false);
  }

  void PoseTransformer::transform_pose(const std::string& sensor_frame,
      const std::string& vehicle_frame,
      const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg_in,
      geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg_out,
      double yaw_std_dev)
  {
    ROS_DEBUG_THROTTLE(10.0, "Sensor frame: %s", sensor_frame.c_str());
    ROS_DEBUG_THROTTLE(10.0, "Vehicle frame: %s", vehicle_frame.c_str());
    ROS_DEBUG_THROTTLE(10.0, "World frame: %s", world_frame_.c_str());

    tf2::Transform sensor_transform;
    geometry_msgs::msg::TransformStamped ssensor_transform;
    *msg_out = *msg_in;

    // Get the static transform defining how the sensor is mounted to the
    // vehicle frame with respect to the localization reference point on the
    // vehicle.
    bool success = false;
    try
    {
      ssensor_transform = tf_buffer_->lookupTransform(
          vehicle_frame,
          sensor_frame,
          tf2::timeFromSec(0),
          tf2::durationFromSec(0));
      success = true;
      tf2::fromMsg(ssensor_transform.transform, sensor_transform);
    }
    catch (const tf2::LookupException& e)
    {
      ROS_WARN("Pose Transformer: Get sensor Transform: %s", e.what());
    }
    catch (const tf2::ConnectivityException& e)
    {
      ROS_WARN("Pose Transformer: Get sensor: %s", e.what());
    }
    catch (const tf2::ExtrapolationException& e)
    {
      ROS_WARN("Pose Transformer: Get sensor: %s", e.what());
    }
    catch (...)
    {
      ROS_WARN("Pose Transformer: Get sensor: Error looking up transform");
    }
    if (!success)
    {
      ROS_WARN("Pose Transformer: No valid sensor transform found, "
          "assuming identity");
      sensor_transform.setIdentity();
    }
    ROS_DEBUG_THROTTLE(10.0, "Sensor offset is (%f, %f, %f)",
        sensor_transform.getOrigin().getX(),
        sensor_transform.getOrigin().getY(),
        sensor_transform.getOrigin().getZ());
    // Get the rotation component of the transform.
    tf2::Transform sensor_rotation(sensor_transform.getRotation());

    // Snap the rotation to be axis aligned for processing the covariances
    // cleanly.  This is to account for the fact that the covariance matrices
    // contain special flag values that can't be blended which would occur for a
    // non-right-angle rotation.
    tf2::Matrix3x3 aligned_rotation(
        swri_transform_util::SnapToRightAngle(sensor_transform.getRotation()));

    // Only try to transform the position data if it is valid since this is the
    // only transform that requires the pose of the vehicle to be known.
    double cov_thresh = std::numeric_limits<double>::max() * 0.4;
    if (msg_in->pose.covariance[0] < cov_thresh ||
        msg_in->pose.covariance[7] < cov_thresh ||
        msg_in->pose.covariance[14] < cov_thresh)
    {
      // Get the vehicle transform with respect to the localization frame.
      tf2::Transform veh_transform;
      geometry_msgs::msg::TransformStamped sveh_transform;
      success = false;
      try
      {
        /*TODO :When the msg stamp is used at the requested time,
        the listener throws an exception because the cache is empty.
        If anyone knows why, they should fix this. */
        sveh_transform = tf_buffer_->lookupTransform(
            world_frame_,
            vehicle_frame,
            tf2::timeFromSec(0),
            tf2::durationFromSec(0));
        tf2::fromMsg(sveh_transform.transform, veh_transform);
        success = true;
      }
      catch (const tf2::LookupException& e)
      {
        ROS_ERROR("Pose Transformer: Get world Transform: %s", e.what());
      }
      catch (const tf2::ConnectivityException& e)
      {
        ROS_ERROR("Pose Transformer: Get world Transform: %s", e.what());
      }
      catch (const tf2::ExtrapolationException& e)
      {
        ROS_ERROR("Pose Transformer: Get world Transform: %s", e.what());
      }
      catch (...)
      {
        ROS_ERROR("Pose Transformer: Get world Transform: "
            "Error looking up transform");
      }
      if (!success)
      {
        ROS_WARN("No valid world transform found, assuming identity");
        veh_transform.setIdentity();
      }

      // Get the rotation component of vehicle transform
      tf2::Matrix3x3 veh_rotation(veh_transform.getRotation());
      ROS_DEBUG_THROTTLE(10.0, "Vehicle rotation is %f (%f, %f, %f)",
          veh_transform.getRotation().getW(),
          veh_transform.getRotation().getX(),
          veh_transform.getRotation().getY(),
          veh_transform.getRotation().getZ());

      // Get the rotated sensor offset transform.
      tf2::Vector3 sensor_offset = sensor_transform.getOrigin();
      tf2::Transform rotated_offset(tf2::Quaternion::getIdentity(),
          veh_rotation * sensor_transform.getOrigin());
      ROS_DEBUG_THROTTLE(10.0, "Rotated Sensor offset is (%f, %f, %f)",
          rotated_offset.getOrigin().getX(),
          rotated_offset.getOrigin().getY(),
          rotated_offset.getOrigin().getZ());

      if (yaw_std_dev < M_PI/6)
      {
        // Transform the position.
        tf2::Vector3 position;
        //tf2::fromMsg(msg_in->pose.pose.position, position);//pointMsgToTf
        position.setX(msg_in->pose.pose.position.x);
        position.setY(msg_in->pose.pose.position.y);
        position.setZ(msg_in->pose.pose.position.z);
        position = rotated_offset.inverse() * position;
        msg_out->pose.pose.position.x = position.x();
        msg_out->pose.pose.position.y = position.y();
        msg_out->pose.pose.position.z = position.z();
        if (0.0 >= yaw_std_dev)
        {
          // Since the pose has been transformed, the variance of the
          // orientation now has an effect on the variance of the position.
          // This effect is non-linear (the distribution goes from being an
          // ellipsoid to a slide of a toroid), but for small angles, we
          // can linearize it and approximate the effect.

          // Calculate the variance in position due to yaw angle error
          double var_yaw = std::pow(sin(yaw_std_dev),2);

          // Create a covariance matrix from the position variance (y-aligned)
          tf2::Matrix3x3 cov_matrix(0,       0, 0,
                                   0, var_yaw, 0,
                                   0,       0, 0);

          // Calculate the inverse of the vehicle rotation matrix
          tf2::Matrix3x3 veh_rotation_inverse = veh_rotation.inverse();

          // Create a rotation matrix corresponding to the sensor offset
          tf2::Matrix3x3 r_sensor_offset(sensor_offset.y(), -sensor_offset.x(), 0,
                                        sensor_offset.x(),  sensor_offset.y(), 0,
                                        0,                                  0, 1);

          // Align covariance with vehicle frame,
          // then apply the inverse of the yaw transform
          cov_matrix = veh_rotation_inverse * r_sensor_offset * cov_matrix.timesTranspose(r_sensor_offset).timesTranspose(veh_rotation_inverse);

          // Add this new covariance component to the existing position
          // covariance matrix
          tf2::Matrix3x3 position_cov = swri_transform_util::GetUpperLeft(msg_in->pose.covariance);
          for (size_t i = 0; i < 9; ++i)
          {
            position_cov[i] += cov_matrix[i];
          }
          // Store new position covariance in message
          swri_transform_util::SetUpperLeft(position_cov, msg_out->pose.covariance);
        }
      }
      else
      {
        // If the yaw error is this large, yaw is completely unreliable.
        // In this case, the center of the vehicle pose distribution is the
        // center of the untransformed pose, and the variances are expanded
        // to encompass the full area that the pose could occupy if it were
        // allowed to "swing" around the full arc of the lever arm of the
        // translation.
        tf2::Matrix3x3 position_cov = swri_transform_util::GetUpperLeft(msg_in->pose.covariance);
        double lever_length_2 = std::pow(sensor_offset.x(), 2) + std::pow(sensor_offset.y(), 2);
        position_cov[0][0] += lever_length_2;
        position_cov[1][1] += lever_length_2;
        swri_transform_util::SetUpperLeft(position_cov, msg_out->pose.covariance);
      }
    }
    // Transform the orientation.
    tf2::Quaternion orientation;
    tf2::fromMsg(msg_in->pose.pose.orientation, orientation);
    orientation = sensor_rotation.inverse() * orientation;
    msg_out->pose.pose.orientation = tf2::toMsg(orientation);

    // Transform the orientation covariance.
    tf2::Matrix3x3 orient_cov =
        swri_transform_util::GetLowerRight(msg_in->pose.covariance);
    orient_cov =
        aligned_rotation.inverse().transposeTimes(orient_cov) * aligned_rotation;
    swri_transform_util::SetLowerRight(orient_cov, msg_out->pose.covariance);
  }
  void PoseTransformer::setWorldFrame(const std::string& world_frame)
  {
    world_frame_ = std::string(world_frame);
  }
  std::string PoseTransformer::getWorldFrame()
  {
    return world_frame_;
  }
} /* namespace sumet_state_estimator */

