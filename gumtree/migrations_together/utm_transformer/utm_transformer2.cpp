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

#include <swri_transform_util/utm_transformer.h>

#include <swri_math_util/trig_util.h>
#include <swri_transform_util/frames.h>

namespace swri_transform_util
{
  UtmTransformer::UtmTransformer() :
    utm_util_(std::make_shared<UtmUtil>()),
    utm_zone_(0),
    utm_band_(0)
  {
  }

  std::map<std::string, std::vector<std::string> > UtmTransformer::Supports() const
  {
    std::map<std::string, std::vector<std::string> >  supports;

    supports[_utm_frame].push_back(_wgs84_frame);
    supports[_wgs84_frame].push_back(_utm_frame);
    supports[_utm_frame].push_back(_tf_frame);
    supports[_tf_frame].push_back(_utm_frame);

    return supports;
  }

  bool UtmTransformer::GetTransform(
    const std::string& target_frame,
    const std::string& source_frame,
    const rclcpp::Time& time,
    Transform& transform)
  {
    if (!initialized_)
    {
      Initialize();
      if (!initialized_)
      {
        return false;
      }
    }

    if (target_frame == _utm_frame)
    {
      if (source_frame == _wgs84_frame)
      {
        transform = std::make_shared<Wgs84ToUtmTransform>(
            time,
            utm_util_,
            utm_zone_,
            utm_band_);

        return true;
      }
      else
      {
        geometry_msgs::msg::TransformStamped tf_transform;
        if (!Transformer::GetTransform(local_xy_frame_, source_frame, time, tf_transform))
        {
          printf("WARN: Failed to get transform from %s to local_xy(%s)",
              source_frame.c_str(), local_xy_frame_.c_str());
          return false;
        }

        transform = std::make_shared<TfToUtmTransform>(
            tf_transform,
            utm_util_,
            local_xy_util_,
            utm_zone_,
            utm_band_);
        return true;
      }
    }
    else if (target_frame == _wgs84_frame && source_frame == _utm_frame)
    {
      transform = std::make_shared<UtmToWgs84Transform>(
          time,
          utm_util_,
          utm_zone_,
          utm_band_);
      return true;
    }
    else if (source_frame == _utm_frame)
    {
      geometry_msgs::msg::TransformStamped tf_transform;
      if (!Transformer::GetTransform(target_frame, local_xy_frame_, time, tf_transform))
      {
        printf("WARN: Failed to get transform from local_xy(%s) to %s",
            local_xy_frame_.c_str(), target_frame.c_str());
        return false;
      }

      transform = std::make_shared<UtmToTfTransform>(
          tf_transform,
          utm_util_,
          local_xy_util_,
          utm_zone_,
          utm_band_);

      return true;
    }

    printf("WARN: Failed to get UTM transform");
    return false;
  }

  bool UtmTransformer::Initialize()
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

    if (initialized_)
    {
      utm_zone_ = GetZone(local_xy_util_->ReferenceLongitude());
      utm_band_ = GetBand(local_xy_util_->ReferenceLatitude());
    }

    return initialized_;
  }

  UtmToTfTransform::UtmToTfTransform(
      const geometry_msgs::msg::TransformStamped& transform,
      std::shared_ptr<UtmUtil> utm_util,
      std::shared_ptr<LocalXyWgs84Util> local_xy_util,
      int32_t utm_zone,
      char utm_band) :
      transform_(transform),
      utm_util_(utm_util),
      local_xy_util_(local_xy_util),
      utm_zone_(utm_zone),
      utm_band_(utm_band)
  {
    stamp_ = transform.header.stamp;
  }

  void UtmToTfTransform::Transform(const tf2::Vector3& v_in, tf2::Vector3& v_out) const
  {
    // Convert to WGS84 latitude and longitude
    double lat, lon;
    utm_util_->ToLatLon(utm_zone_, utm_band_, v_in.x(), v_in.y(), lat, lon);

    // Convert to the LocalXY coordinate system.
    double x, y;
    local_xy_util_->ToLocalXy(lat, lon, x, y);

    // Transform from the LocalXY coordinate frame using the TF transform
    v_out.setValue(x, y, v_in.z());
    tf2::Transform tf;
    tf2::fromMsg(transform_.transform, tf);
    v_out = tf * v_out;
  }

  tf2::Quaternion UtmToTfTransform::GetOrientation() const
  {
    tf2::Quaternion reference_angle;
    reference_angle.setRPY(0, 0,
      swri_math_util::ToRadians(local_xy_util_->ReferenceAngle()));

    tf2::Transform tf;
    tf2::fromMsg(transform_.transform, tf);
    return tf.getRotation() * reference_angle.inverse();
  }

  TransformImplPtr UtmToTfTransform::Inverse() const
  {
    geometry_msgs::msg::TransformStamped inverse_transform = transform_;
    tf2::Transform tf;
    tf2::fromMsg(transform_.transform, tf);
    inverse_transform.transform = tf2::toMsg(tf.inverse());
    inverse_transform.header.frame_id = transform_.child_frame_id;
    inverse_transform.child_frame_id = transform_.header.frame_id;
    TransformImplPtr inverse = std::make_shared<TfToUtmTransform>(
        inverse_transform,
        utm_util_,
        local_xy_util_,
        utm_zone_,
        utm_band_);
    //inverse->stamp_ = stamp_;
    return inverse;
  }

  TfToUtmTransform::TfToUtmTransform(
      const geometry_msgs::msg::TransformStamped& transform,
      std::shared_ptr<UtmUtil> utm_util,
      std::shared_ptr<LocalXyWgs84Util> local_xy_util,
      int32_t utm_zone,
      char utm_band) :
      transform_(transform),
      utm_util_(utm_util),
      local_xy_util_(local_xy_util),
      utm_zone_(utm_zone),
      utm_band_(utm_band)
  {
    stamp_ = transform.header.stamp;
  }

  void TfToUtmTransform::Transform(const tf2::Vector3& v_in, tf2::Vector3& v_out) const
  {
    // Transform into the LocalXY coordinate frame using the TF transform
    tf2::Transform tf;
    tf2::fromMsg(transform_.transform, tf);
    tf2::Vector3 local_xy = tf * v_in;

    // Convert to WGS84 latitude and longitude
    double latitude, longitude;
    local_xy_util_->ToWgs84(local_xy.x(), local_xy.y(), latitude, longitude);

    // Convert to UTM easting and northing
    double easting, northing;
    utm_util_->ToUtm(latitude, longitude, easting, northing);
    v_out.setValue(easting, northing, local_xy.z());
  }

  tf2::Quaternion TfToUtmTransform::GetOrientation() const
  {
    tf2::Quaternion reference_angle;
    reference_angle.setRPY(0, 0,
      swri_math_util::ToRadians(local_xy_util_->ReferenceAngle()));

    tf2::Transform tf;
    tf2::fromMsg(transform_.transform, tf);
    return tf.getRotation() * reference_angle;
  }

  TransformImplPtr TfToUtmTransform::Inverse() const
  {
    geometry_msgs::msg::TransformStamped inverse_transform = transform_;
    tf2::Transform tf;
    tf2::fromMsg(transform_.transform, tf);
    inverse_transform.transform = tf2::toMsg(tf.inverse());
    inverse_transform.header.frame_id = transform_.child_frame_id;
    inverse_transform.child_frame_id = transform_.header.frame_id;
    TransformImplPtr inverse = std::make_shared<UtmToTfTransform>(
        inverse_transform,
        utm_util_,
        local_xy_util_,
        utm_zone_,
        utm_band_);
    //inverse->stamp_ = stamp_;
    return inverse;
  }

  UtmToWgs84Transform::UtmToWgs84Transform(
    const builtin_interfaces::msg::Time& time,
    std::shared_ptr<UtmUtil> utm_util,
    int32_t utm_zone,
    char utm_band) :
    utm_util_(utm_util),
    utm_zone_(utm_zone),
    utm_band_(utm_band)
  {
    stamp_ = time;//handle_->now();
  }

  void UtmToWgs84Transform::Transform(const tf2::Vector3& v_in, tf2::Vector3& v_out) const
  {
    double lat, lon;
    utm_util_->ToLatLon(utm_zone_, utm_band_, v_in.x(), v_in.y(), lat, lon);
    v_out.setValue(lon, lat, v_in.z());
  }

  TransformImplPtr UtmToWgs84Transform::Inverse() const
  {
    TransformImplPtr inverse = std::make_shared<Wgs84ToUtmTransform>(
        stamp_,
        utm_util_,
        utm_zone_,
        utm_band_);
    //inverse->stamp_ = stamp_;
    return inverse;
  }


  Wgs84ToUtmTransform::Wgs84ToUtmTransform(
    const builtin_interfaces::msg::Time& time,
    std::shared_ptr<UtmUtil> utm_util,
    int32_t utm_zone,
    char utm_band) :
    utm_util_(utm_util),
    utm_zone_(utm_zone),
    utm_band_(utm_band)
  {
    stamp_ = time;//handle_->now();
  }

  void Wgs84ToUtmTransform::Transform(const tf2::Vector3& v_in, tf2::Vector3& v_out) const
  {
    double easting, northing;
    utm_util_->ToUtm(v_in.y(), v_in.x(), easting, northing);
    v_out.setValue(easting, northing, v_in.z());
  }

  TransformImplPtr Wgs84ToUtmTransform::Inverse() const
  {
    TransformImplPtr inverse = std::make_shared<UtmToWgs84Transform>(
        stamp_,
        utm_util_,
        utm_zone_,
        utm_band_);
    //inverse->stamp_ = stamp_;
    return inverse;
  }
}
