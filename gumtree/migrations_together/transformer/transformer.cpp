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

#include <swri_transform_util/transformer.h>

namespace swri_transform_util
{
  Transformer::Transformer() : initialized_(false)
  {
  }

  Transformer::~Transformer()
  {
  }

  void Transformer::Initialize(
      const boost::shared_ptr<tf::TransformListener> tf)
  {
    tf_listener_ = tf;
    initialized_ = Initialize();
  }

  bool Transformer::Initialize()
  {
    return true;
  }

  bool Transformer::GetTransform(
      const std::string& target_frame,
      const std::string& source_frame,
      const ros::Time& time,
      tf::StampedTransform& transform) const
  {
    if (!tf_listener_)
    {
      return false;
    }

    bool has_transform = false;
    try
    {
      if (tf_listener_->frameExists(target_frame) &&
          tf_listener_->frameExists(source_frame) &&
          tf_listener_->waitForTransform(
          target_frame,
          source_frame,
          time,
          ros::Duration(0.01)))
      {
        tf_listener_->lookupTransform(
            target_frame,
            source_frame,
            time,
            transform);

        has_transform = true;
      }
    }
    catch (const tf::LookupException& e)
    {
      ROS_ERROR_THROTTLE(2.0, "[transformer]: %s", e.what());
    }
    catch (const tf::ConnectivityException& e)
    {
      ROS_ERROR_THROTTLE(2.0, "[transformer]: %s", e.what());
    }
    catch (const tf::ExtrapolationException& e)
    {
      ROS_ERROR_THROTTLE(2.0, "[transformer]: %s", e.what());
    }
    catch (...)
    {
      ROS_ERROR_THROTTLE(2.0, "[transformer]: Exception looking up transform");
    }

    return has_transform;
  }
}
