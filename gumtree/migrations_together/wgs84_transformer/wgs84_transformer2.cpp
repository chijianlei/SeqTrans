// *****************************************************************************
//
// Copyright (c) 2014, Southwest Research Institute® (SwRI®)
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Southwest Research Institute® (SwRI®) nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// *****************************************************************************

#include <swri_transform_util/wgs84_transformer.h>

//#include <boost/make_shared.hpp>

#include <swri_math_util/trig_util.h>
#include <swri_transform_util/frames.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace swri_transform_util
{
  Wgs84Transformer::Wgs84Transformer()
  {
  }

  std::map<std::string, std::vector<std::string> > Wgs84Transformer::Supports() const
  {
    std::map<std::string, std::vector<std::string> >  supports;

    supports[_wgs84_frame].push_back(_tf_frame);
    supports[_tf_frame].push_back(_wgs84_frame);

    return supports;
  }

  bool Wgs84Transformer::GetTransform(
    const std::string& target_frame,
    const std::string& source_frame,
    const rclcpp::Time& time,
    Transform& transform)
  {
    if (!initialized_)
    {
      Initialize();
    }

    if (!initialized_)
    {
      printf("WARN: Wgs84Transformer not initialized");
      return false;
    }

    if (target_frame == _wgs84_frame)
    {
      geometry_msgs::msg::TransformStamped tf_transform;
      if (!Transformer::GetTransform(local_xy_frame_, source_frame , time, tf_transform))
      {
        printf("WARN: Failed to get transform between %s and %s",
            source_frame.c_str(), local_xy_frame_.c_str());
        return false;
      }

      transform = std::make_shared<TfToWgs84Transform>(tf_transform, local_xy_util_);

      return true;
    }
    else if (source_frame == _wgs84_frame)
    {
      geometry_msgs::msg::TransformStamped tf_transform;
      if (!Transformer::GetTransform(target_frame, local_xy_frame_, time, tf_transform))
      {
        printf("WARN: Failed to get transform between %s and %s",
            local_xy_frame_.c_str(), target_frame.c_str());
        return false;
      }

      transform = std::make_shared<Wgs84ToTfTransform>(tf_transform, local_xy_util_);

      return true;
    }

    printf("WARN: Failed to get WGS84 transform.");
    return false;
  }

  bool Wgs84Transformer::Initialize()
  {
    if (!local_xy_util_)
    {
      local_xy_util_ = std::make_shared<LocalXyWgs84Util>(handle_);
    }

    if (local_xy_util_->Initialized())
    {
      std::string local_xy_frame = local_xy_util_->Frame();
      if (tf_listener_->_frameExists(local_xy_frame))
      {
        local_xy_frame_ = local_xy_frame;
        initialized_ = true;
      }
      else if (!local_xy_frame.empty() && local_xy_frame[0] == '/' && tf_listener_->_frameExists(local_xy_frame.substr(1)))
      {
        local_xy_frame_ = local_xy_frame.substr(1);
        initialized_ = true;
      }
      else if (!local_xy_frame.empty() && local_xy_frame[0] != '/' && tf_listener_->_frameExists("/" + local_xy_frame))
      {
        local_xy_frame_ = "/" + local_xy_frame;
        initialized_ = true;
      }
    }

    return initialized_;
  }
  
  TfToWgs84Transform::TfToWgs84Transform(
    const geometry_msgs::msg::TransformStamped& transform,
    std::shared_ptr<LocalXyWgs84Util> local_xy_util) :
    transform_(transform),
    local_xy_util_(local_xy_util)
  {
    //stamp_ = transform.stamp_;
    //int64_t count = tf2::Duration(transform.stamp_.time_since_epoch()).count();

    // scale the nanoseconds separately for improved accuracy
    //int32_t sec, nsec;
    //nsec = static_cast<int32_t>(count % 1000000000l);
    //sec = static_cast<int32_t>((count - nsec) / 1000000000l);
    stamp_ = transform.header.stamp;
  }

  void TfToWgs84Transform::Transform(const tf2::Vector3& v_in, tf2::Vector3& v_out) const
  {
    // Transform into the LocalXY coordinate frame using the TF transform.
    tf2::Transform tf;
    tf2::fromMsg(transform_.transform, tf);
    tf2::Vector3 local_xy = tf * v_in;

    // Convert to WGS84 latitude and longitude.
    double latitude, longitude;
    local_xy_util_->ToWgs84(local_xy.x(), local_xy.y(), latitude, longitude);
    v_out.setValue(longitude, latitude, local_xy.z());
  }
  
  tf2::Quaternion TfToWgs84Transform::GetOrientation() const
  {
    tf2::Quaternion reference_angle;
    reference_angle.setRPY(0, 0,
      swri_math_util::ToRadians(local_xy_util_->ReferenceAngle()));
 
    tf2::Transform tf;
    tf2::fromMsg(transform_.transform, tf);
    return tf.getRotation() * reference_angle;
  }

  TransformImplPtr TfToWgs84Transform::Inverse() const
  {
    geometry_msgs::msg::TransformStamped inverse_transform = transform_;
    tf2::Transform tf;
    tf2::fromMsg(transform_.transform, tf);
    inverse_transform.transform = tf2::toMsg(tf.inverse());
    inverse_transform.header.frame_id = transform_.child_frame_id;
    inverse_transform.child_frame_id = transform_.header.frame_id;
    TransformImplPtr inverse = std::make_shared<Wgs84ToTfTransform>(
        inverse_transform,
        local_xy_util_);
    inverse->stamp_ = stamp_;
    return inverse;
  }
  
  Wgs84ToTfTransform::Wgs84ToTfTransform(
    const geometry_msgs::msg::TransformStamped& transform,
    std::shared_ptr<LocalXyWgs84Util> local_xy_util) :
    transform_(transform),
    local_xy_util_(local_xy_util)
  {
    stamp_ = transform.header.stamp;
  }

  void Wgs84ToTfTransform::Transform(const tf2::Vector3& v_in, tf2::Vector3& v_out) const
  {
    // Convert to LocalXY coordinate frame.
    double x, y;
    local_xy_util_->ToLocalXy(v_in.y(), v_in.x(), x, y);
    v_out.setValue(x, y, v_in.z());

    // Transform from the LocalXY coordinate frame using the TF transform.
    tf2::Transform tf;
    tf2::fromMsg(transform_.transform, tf);
    v_out = tf * v_out;
  }
  
  tf2::Quaternion Wgs84ToTfTransform::GetOrientation() const
  {
    tf2::Quaternion reference_angle;
    reference_angle.setRPY(0, 0,
      swri_math_util::ToRadians(local_xy_util_->ReferenceAngle()));
      
    tf2::Transform tf;
    tf2::fromMsg(transform_.transform, tf);
    return tf.getRotation() * reference_angle.inverse();
  }

  TransformImplPtr Wgs84ToTfTransform::Inverse() const
  {
    geometry_msgs::msg::TransformStamped inverse_transform = transform_;
    tf2::Transform tf;
    tf2::fromMsg(transform_.transform, tf);
    inverse_transform.transform = tf2::toMsg(tf.inverse());
    inverse_transform.header.frame_id = transform_.child_frame_id;
    inverse_transform.child_frame_id = transform_.header.frame_id;
    TransformImplPtr inverse = std::make_shared<TfToWgs84Transform>(
        inverse_transform,
        local_xy_util_);
    inverse->stamp_ = stamp_;
    return inverse;
  }
}

