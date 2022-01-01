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
 * VelocityStreamList3D.cpp
 *
 *  Created on: Feb 16, 2012
 *      Author: kkozak
 */

#include <sumet_state_estimator/VelocityStreamList3D.h>

#include <string>
#include <vector>

namespace sumet_state_estimator
{
    VelocityStreamList3D::VelocityStreamList3D()
    {
      init();
    }

    void VelocityStreamList3D::set_averaging_window(double win)
    {
      static_window_ = win;
      // We need to make sure that we propagate the window (if it's changed
      // when data is loaded) to the lowest levels)
      StreamMap3D::iterator it;
      for (it = StreamArray_.begin(); it != StreamArray_.end(); ++it)
      {
        it->second.set_static_window(win);
      }
    }

    bool VelocityStreamList3D::set_stream_averaging_window(
      const std::string& key,
      double win)
    {
      if (!in_map(key))
      {
        return false;
      }

      StreamArray_[key].set_static_window(win);
      return true;
    }


    bool VelocityStreamList3D::init_add_stream_def(
      const std::string& key,
      sumet_state_estimator::VelPriority priority,
      int capacity_of_buffer)
    {
      std::vector<bool> v_is_valid;
      std::vector<bool> w_is_valid;
      for (uint32_t i = 0; i < 3; ++i)
      {
        v_is_valid.push_back(false);
        w_is_valid.push_back(false);
      }
      if (!in_map(key))
      {
        create_new_stream(key,
                          v_is_valid,
                          w_is_valid,
                          priority,
                          capacity_of_buffer);
        return true;
      }
      else
      {
        return false;
      }
    }


    bool VelocityStreamList3D::init_add_stream_def(
      const std::string& key,
      std::vector<bool> v_is_valid,
      std::vector<bool> w_is_valid,
      sumet_state_estimator::VelPriority priority,
      int capacity_of_buffer)
    {
      if (!in_map(key))
      {
        create_new_stream(key,
                          v_is_valid,
                          w_is_valid,
                          priority,
                          capacity_of_buffer);
        return true;
      }
      else
      {
        return false;
      }
    }


    bool VelocityStreamList3D::init_add_stream_def(
      const std::string& key,
      std::vector<bool> v_is_valid,
      std::vector<bool> w_is_valid,
      sumet_state_estimator::VelPriority priority,
      double win,
      int capacity_of_buffer)
    {
      bool success = init_add_stream_def(key,
                                         v_is_valid,
                                         w_is_valid,
                                         priority,
                                         capacity_of_buffer);

      set_stream_averaging_window(key,
                                  win);

      return success;
    }


    void VelocityStreamList3D::get_streams(
      std::vector< std::vector< std::string> >& averaging_menu,
      const rclcpp::Time& t_m,
      velocity_type vel_type)
    {
      // TODO(kkozak): The VelIgnore level does not appear to be ignored
      std::vector< sumet_state_estimator::VelPriority> priority_vec;
      averaging_menu.clear();

      // Initialize the averaging menu variable
      for (uint32_t i = 0; i < 3; ++i)
      {
        std::vector<std::string> temp;
        averaging_menu.push_back(temp);
        priority_vec.push_back(sumet_state_estimator::VelIgnore);
      }

      // Iterate over all of the streams and find the highest priority samples
      // available for each component
      StreamMap3D::iterator it;
      for (it = StreamArray_.begin(); it != StreamArray_.end(); ++it)
      {
        // Get the priority and validity for the current stream
        sumet_state_estimator::VelPriority priority = it->second.get_priority();
        std::vector<bool> valid;
        if (LINEAR_VELOCITY == vel_type)
        {
          valid = it->second.v_is_valid();
        }
        else if (ANGULAR_VELOCITY == vel_type)
        {
          valid = it->second.w_is_valid();
        }
        else
        {
          ROS_ERROR("get_streams failed: velocity_type invalid: "
                    "val = %d, should be either %d or %d",
                    vel_type, LINEAR_VELOCITY, ANGULAR_VELOCITY);
        }

        // Iterate over all 3 components
        for (uint32_t i = 0; i < valid.size(); i++)
        {
          // We only need to add it if the sample is valid and in the window
          if (valid[i] && it->second.samples_in_window(t_m))
          {
            // If the current priority is higher (lower numerically)
            // replace whatever is currently there (if anything)
            if (priority < priority_vec[i])
            {
              priority_vec[i] = priority;
              averaging_menu[i].clear();
              averaging_menu[i].push_back(it->first);
            }
            else if (priority == priority_vec[i])
            {
              // If the current priority is the same, then we'll just add this
              // stream to the list
              averaging_menu[i].push_back(it->first);

              if (priority_vec[i] == sumet_state_estimator::VelOnly)
              {
                // Because of the exclusivity of the VelOnly priority, we want
                // to make sure that we're not trying to add more than one at
                // this level (it probably indicates an error, so we'll print
                // out an error message if that occurs).
                char msg1[1024];

                snprintf(msg1, sizeof(msg1),
                        "\nMore than one velocity stream of same type listed "
                        "as the exclusive stream on dimension %d for "
                        "averaging. Current stream is %s\n",
                        i,
                        it->first.c_str());

                std::string err_msg(msg1);
                for (uint32_t j = 0; j < averaging_menu[i].size(); j++)
                {
                  snprintf(msg1, sizeof(msg1),
                          "%s\n",
                          averaging_menu[i][j].c_str());
                }
                err_msg += msg1;
                ROS_ERROR("%s",
                          err_msg.c_str());
              }
            }
          }
        }
      }
    }

    void VelocityStreamList3D::get_w_streams(
      std::vector< std::vector< std::string> >& averaging_menu,
      const rclcpp::Time& t_m)
    {
      get_streams(averaging_menu,
                  t_m,
                  ANGULAR_VELOCITY);
    }



     void VelocityStreamList3D::get_v_streams(
       std::vector< std::vector< std::string> >& averaging_menu,
       const rclcpp::Time& t_m)
     {
       get_streams(averaging_menu,
                   t_m,
                   LINEAR_VELOCITY);
     }



     void VelocityStreamList3D::get_averaged_twist(
       const rclcpp::Time& t_m,
       geometry_msgs::msg::TwistWithCovariance& twist_out)
     {
       tf2::Vector3 v_mean;
       tf2::Vector3 v_var;
       tf2::Vector3 w_mean;
       tf2::Vector3 w_var;
       std::vector<bool> v_valid = get_v_average(t_m, v_mean, v_var);
       std::vector<bool> w_valid = get_w_average(t_m, w_mean, w_var);
       std::array<double, 36>& cov = twist_out.covariance;
       for (uint32_t i = 0; i < 36; i++)
       {
         cov[i] = 0.0;
       }

       for (uint32_t i = 0; i < 3; i++)
       {
         if (!v_valid[i])
         {
           v_mean.m_floats[i] = 0.0;
           cov[i*7] = 1e20;
         }
         else
         {
           cov[i*7] = v_var.m_floats[i];
         }
         if (!w_valid[i])
         {
           w_mean.m_floats[i] = 0.0;
           cov[21 + i*7] = 1e20;
         }
         else
         {
           cov[21 + i*7] = w_var[i];
         }
       }
       twist_out.twist.linear.x = v_mean.x();
       twist_out.twist.linear.y = v_mean.y();
       twist_out.twist.linear.z = v_mean.z();

       twist_out.twist.angular.x = w_mean.x();
       twist_out.twist.angular.y = w_mean.y();
       twist_out.twist.angular.z = w_mean.z();
     }

     std::vector<bool> VelocityStreamList3D::do_average(
       std::vector< std::vector< std::string> >& averaging_menu,
       const rclcpp::Time& t_m,
       tf2::Vector3& v_mean,
       tf2::Vector3& v_var,
       velocity_type vel_type)
     {
       std::vector<bool> is_valid;
       for (uint32_t i = 0; i < 3; i++)
       {
         is_valid.push_back(false);
       }

       v_mean.setZero();
       v_var.setZero();

       std::vector<bool> valid_elems;
       for (uint32_t i = 0; i < 3; i++)
       {
         std::vector< std::string > keys = averaging_menu[i];

         std::vector<double> averages;
         std::vector<double> vars;
         for (uint32_t j = 0; j < keys.size(); j++)
         {
           tf2::Vector3 temp_vel;
           tf2::Vector3 temp_var;
           if (LINEAR_VELOCITY == vel_type)
           {
             valid_elems = StreamArray_[keys[j]].do_vel_average(temp_vel,
                                                                temp_var,
                                                                t_m);
           }
           else if (ANGULAR_VELOCITY == vel_type)
           {
             valid_elems = StreamArray_[keys[j]].do_w_average(temp_vel,
                                                              temp_var,
                                                              t_m);
           }
           else
           {
             ROS_ERROR("do_average failed: velocity_type invalid: "
                       "val = %d, should be either %d or %d",
                       vel_type, LINEAR_VELOCITY, ANGULAR_VELOCITY);
             valid_elems.clear();
             valid_elems.push_back(false);
             valid_elems.push_back(false);
             valid_elems.push_back(false);
           }
           // We should have already made sure that the current element is
           // valid, but we'll check here to be sure
           if (!valid_elems[i])
           {
             ROS_ERROR("Trying to take an average of a non-valid linear "
                       "velocity stream: key = %s, dim = %d",
                       keys[j].c_str(),
                       i);
           }
           else
           {
             averages.push_back(temp_vel[i]);
             vars.push_back(temp_var[i]);
           }
         }
         if (!averages.empty())
         {
           is_valid[i] = true;

           double inv_var_sum = 0.0;
           for (uint32_t j = 0; j < averages.size(); j++)
           {
             v_mean.m_floats[i] += averages[j] / vars[j];
             inv_var_sum += 1/vars[j];
           }
           v_mean.m_floats[i] /= inv_var_sum;
           v_var.m_floats[i] = 1.0 / inv_var_sum;
         }
       }
       return is_valid;
     }

     std::vector<bool> VelocityStreamList3D::get_v_average(
       const rclcpp::Time& t_m,
       tf2::Vector3& v_mean,
       tf2::Vector3& v_var)
     {
       std::vector< std::vector< std::string> > averaging_menu;

       get_v_streams(averaging_menu,
                     t_m);

       std::vector<bool> is_valid = do_average(averaging_menu,
                                               t_m,
                                               v_mean,
                                               v_var,
                                               LINEAR_VELOCITY);
       return is_valid;
     }


     std::vector<bool> VelocityStreamList3D::get_w_average(
       const rclcpp::Time& t_m,
       tf2::Vector3& w_mean,
       tf2::Vector3& w_var)
     {
       std::vector< std::vector< std::string> > averaging_menu;

       get_w_streams(averaging_menu,
                     t_m);

       std::vector<bool> is_valid = do_average(averaging_menu,
                                               t_m,
                                               w_mean,
                                               w_var,
                                               ANGULAR_VELOCITY);
       return is_valid;
     }


    void VelocityStreamList3D::load_new_data(
      const std::string& key,
      const sumet_state_estimator::VelocityElem3D& new_elem)
    {
      if (!in_map(key))
      {
        ROS_ERROR("Key for non-initialized VelocityStream Used.  "
            "A new stream definition was automatically added.");
        init_add_stream_def(key,
                            sumet_state_estimator::VelIgnore);
      }
      StreamArray_[key].add_v_elem(new_elem);
    }


    bool VelocityStreamList3D::get_stream_validity(
      const std::string& key,
      std::vector<bool>& v_valid, std::vector<bool>& w_valid)
    {
      if (!stream_exists(key))
      {
        return false;
      }

      v_valid = StreamArray_[key].v_is_valid();
      w_valid = StreamArray_[key].w_is_valid();

      return true;
    }

    bool VelocityStreamList3D::get_stream_priority(
      const std::string& key,
      sumet_state_estimator::VelPriority& priority)
    {
      if (!stream_exists(key))
      {
        return false;
      }
      priority = StreamArray_[key].get_priority();
      return true;
    }

    void VelocityStreamList3D::set_stream_priority(
      const std::string& key,
      sumet_state_estimator::VelPriority priority)
    {
      if (!in_map(key))
      {
        ROS_ERROR("Key not found.  Cannot set stream priority");
        return;
      }
      StreamArray_[key].set_priority(priority);
    }

    bool VelocityStreamList3D::stream_exists(const std::string& key)
    {
      return in_map(key);
    }

    void VelocityStreamList3D::print_stream_array()
    {
      StreamMap3D::iterator it;
      for (it = StreamArray_.begin(); it != StreamArray_.end(); ++it)
      {
        it->second.print_list(it->first);
      }
    }

    void VelocityStreamList3D::init(double win)
    {
      StreamArray_.clear();
      static_window_ = win;
    }

    void VelocityStreamList3D::create_new_stream(
      const std::string& key,
      std::vector<bool> v_is_valid,
      std::vector<bool> w_is_valid,
      sumet_state_estimator::VelPriority priority,
      int capacity_of_buffer)
    {
      VelocityList3D temp_list;

      temp_list.init(v_is_valid,
                     w_is_valid,
                     capacity_of_buffer,
                     static_window_,
                     priority);

      StreamArray_[key] = temp_list;
    }

    bool VelocityStreamList3D::in_map(const std::string& key)
    {
      StreamMap3D::iterator it = StreamArray_.find(key);
      if (it != StreamArray_.end())
      {
        return true;
      }
      return false;
    }
}
