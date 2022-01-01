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

/*
 * VelocityList3D.cpp
 *
 *  Created on: Feb 16, 2012
 *      Author: kkozak
 */

#include <sumet_state_estimator/VelocityList3D.h>

#include <string>
#include <algorithm>
#include <vector>

namespace sumet_state_estimator
{
    VelocityList3D::VelocityList3D()
    {
      init();
    }

    void VelocityList3D::init(int capacity_of_buffer,
                              double static_window,
                              sumet_state_estimator::VelPriority priority)
    {
      std::vector<bool> v_is_valid;
      std::vector<bool> w_is_valid;
      for (uint32_t i = 0; i < 3; i++)
      {
        v_is_valid.push_back(false);
        w_is_valid.push_back(false);
      }
      init(v_is_valid,
           w_is_valid,
           capacity_of_buffer,
           static_window,
           priority);
    }


    void VelocityList3D::init(std::vector<bool> v_is_valid,
                              std::vector<bool> w_is_valid,
                              int capacity_of_buffer,
                              double static_window,
                              sumet_state_estimator::VelPriority priority)
    {
      set_buffer_capacity(capacity_of_buffer);
      set_v_valid(v_is_valid);
      set_w_valid(w_is_valid);
      set_static_window(static_window);

      priority_ = priority;
    }


    void VelocityList3D::set_priority(VelPriority priority)
    {
      priority_ = priority;
    }


    sumet_state_estimator::VelPriority VelocityList3D::get_priority()
    {
      return priority_;
    }


    void VelocityList3D::set_buffer_capacity(int capacity_of_buffer)
    {
      // Clear out the buffer when we set capacity to avoid old items
      // from persisting.
      VelocityStream_.clear();
      VelocityStream_.resize_list(capacity_of_buffer);
    }


    void VelocityList3D::set_v_valid(std::vector<bool> v_valid)
    {
      v_is_valid_ = v_valid;
    }


    void VelocityList3D::set_w_valid(std::vector<bool> w_valid)
    {
      w_is_valid_ = w_valid;
    }


    std::vector<bool> VelocityList3D::v_is_valid()
    {
      return v_is_valid_;
    }


    std::vector<bool> VelocityList3D::w_is_valid()
    {
      return w_is_valid_;
    }


    void VelocityList3D::set_static_window(double win)
    {
      static_window_ = win;
    }


    void VelocityList3D::add_v_elem(
        const sumet_state_estimator::VelocityElem3D& new_elem)
    {
      set_v_valid(new_elem.get_v_is_valid());
      set_w_valid(new_elem.get_w_is_valid());
      VelocityStream_.push_back(new_elem);
    }


    void VelocityList3D::add_v(tf::Vector3 v_in,
                               tf::Vector3 v_var_in,
                               std::vector<bool> v_is_valid,
                               const ros::Time& timestamp)
    {
      set_v_valid(v_is_valid);
      sumet_state_estimator::VelocityElem3D temp_elem;

      temp_elem.load_v(v_in,
                       v_var_in,
                       v_is_valid);

      temp_elem.header.stamp = timestamp;
      add_v_elem(temp_elem);
    }


    void VelocityList3D::add_w(tf::Vector3 w_in,
                               tf::Vector3 w_var_in,
                               std::vector<bool> w_is_valid,
                               const ros::Time& timestamp)
    {
      set_w_valid(w_is_valid);
      sumet_state_estimator::VelocityElem3D temp_elem;

      temp_elem.load_w(w_in,
                       w_var_in,
                       w_is_valid);

      temp_elem.header.stamp = timestamp;
      add_v_elem(temp_elem);
    }


    void VelocityList3D::add_v_and_w(tf::Vector3 v_in,
                                     tf::Vector3 v_var_in,
                                     std::vector<bool> v_is_valid,
                                     tf::Vector3 w_in,
                                     tf::Vector3 w_var_in,
                                     std::vector<bool> w_is_valid,
                                     const ros::Time& timestamp)
    {
      set_v_valid(v_is_valid);
      set_w_valid(w_is_valid);

      sumet_state_estimator::VelocityElem3D temp_elem;
      temp_elem.load_v(v_in,
                       v_var_in,
                       v_is_valid);

      temp_elem.load_w(w_in,
                       w_var_in,
                       w_is_valid);

      temp_elem.header.stamp = timestamp;

      add_v_elem(temp_elem);
    }


    std::vector<bool> VelocityList3D::do_average(tf::Vector3& avg_vel,  // NOLINT
                                                 tf::Vector3& var,
                                                 const ros::Time& t_m,
                                                 velocity_type vel_type)
    {
      ros::Time T1 = t_m - ros::Duration(static_window_ / 2.0);
      ros::Time T2 = t_m + ros::Duration(static_window_ / 2.0);
      int idxLow = -1;
      int idxHigh = -1;
      std::vector<bool> v_is_valid;

      std::vector<bool> temp_validity;
      if (LINEAR_VELOCITY == vel_type)
      {
        temp_validity = v_is_valid_;
      }
      else if (ANGULAR_VELOCITY == vel_type)
      {
        temp_validity = w_is_valid_;
      }
      else
      {
        ROS_ERROR("Velocity type incorrect in do_average (VelocityList3D)."
                  "Expecting %d or %d, got %d",
                  LINEAR_VELOCITY, ANGULAR_VELOCITY, vel_type);
      }

      for (uint32_t i = 0; i < 3; i++)
      {
        v_is_valid.push_back(false);
      }

      if (VelocityStream_.find_idxs_in_time_range(idxLow, idxHigh, T1, T2))
      {
        if (idxHigh < idxLow)
        {
          return v_is_valid;
        }

        for (uint32_t j = 0; j < 3; j++)
        {
          if (temp_validity[j])
          {
            double avg_vel_temp = 0.0;
            double num = 0.0;
            double var_temp = 1e20;
            for (int i = idxLow; i <= idxHigh; ++i)
            {
              // TODO(kkozak): Need to fix the std (don't just take
              // the most recent one), when the envelope is added to
              // the window.  Some testing may be required to
              // determine how averaging affects the error in order to
              // do this.
              tf::Vector3 var1;
              tf::Vector3 temp;
              if (LINEAR_VELOCITY == vel_type)
              {
                temp = VelocityStream_[i].get_v(var1,
                                                v_is_valid);
              }
              else if (ANGULAR_VELOCITY == vel_type)
              {
                temp = VelocityStream_[i].get_w(var1,
                                                v_is_valid);
              }

              num += temp.m_floats[j];
              var_temp = var1.m_floats[j];
            }

            int N = (idxHigh - idxLow) + 1;
            avg_vel_temp = num / static_cast<double>(N);
            avg_vel.m_floats[j] = avg_vel_temp;
            var.m_floats[j] = var_temp;
          }
        }
      }

      return v_is_valid;
    }


    std::vector<bool> VelocityList3D::do_vel_average(tf::Vector3& avg_vel,  // NOLINT
                                                     tf::Vector3& var,
                                                     const ros::Time& t_m)
    {
      std::vector<bool> v_is_valid;
      v_is_valid = do_average(avg_vel,
                              var,
                              t_m,
                              LINEAR_VELOCITY);
      return v_is_valid;
    }



    std::vector<bool> VelocityList3D::do_w_average(tf::Vector3& avg_vel,  // NOLINT
                                                   tf::Vector3& var,
                                                   const ros::Time& t_m)
    {
      std::vector<bool> w_is_valid;
      w_is_valid = do_average(avg_vel,
                              var,
                              t_m,
                              ANGULAR_VELOCITY);
      return w_is_valid;
    }


    bool VelocityList3D::samples_in_window(const ros::Time& t_m)
    {
      ros::Time T1 = t_m - ros::Duration(static_window_ / 2.0);
      ros::Time T2 = t_m + ros::Duration(static_window_ / 2.0);
      int idxLow = -1;
      int idxHigh = -1;
      bool success = VelocityStream_.find_idxs_in_time_range(idxLow,
                                                             idxHigh,
                                                             T1,
                                                             T2);
      if (idxHigh < idxLow)
      {
        success = false;
      }
      return success;
    }


    double VelocityList3D::get_static_window() const
    {
      return static_window_;
    }


    void VelocityList3D::print_list(const std::string& name)
    {
      int N = VelocityStream_.size();
      if (N <= 0)
      {
        return;
      }
      std::vector<bool> valid_temp;
      tf::Vector3 var_temp;
      fprintf(stderr, "%s =\n", name.c_str());
      fprintf(stderr, "timestamp   ");
      fprintf(stderr, "   vx      vy      vz   ");
      fprintf(stderr, "   wx      wy      wz   ");
      fprintf(stderr, "\n");
      for (int i = std::max(0, N-10); i < N; ++i)
      {
        fprintf(stderr, "%f,", VelocityStream_[i].header.stamp.toSec());

        tf::Vector3 v = VelocityStream_[i].get_v(var_temp,
                                               valid_temp);

        tf::Vector3 w = VelocityStream_[i].get_w(var_temp,
                                               valid_temp);

        fprintf(stderr, "  %f    %f    %f  ", v.x(), v.y(), v.z());
        fprintf(stderr, "  %f    %f    %f  ", w.x(), w.y(), w.z());
        fprintf(stderr, "\n");
      }
    }
}
