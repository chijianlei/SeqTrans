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
 * LocalizationQueue6DOF.cpp
 *
 *  Created on: March 28, 2012
 *      Author: kkozak
 */
#include<sumet_state_estimator/LocalizationQueue6DOF.h>

#include <swri_roscpp/logging.h>
#include <swri_roscpp/time.h>

namespace sumet_state_estimator
{
  //////////////////////////////////////////////////////////////////////////////
  //
  //  LocalizationQueue()
  //
  //////////////////////////////////////////////////////////////////////////////
  LocalizationQueue6DOF::LocalizationQueue6DOF()
  {
    initialize_queue();
  }

  //////////////////////////////////////////////////////////////////////////////
  //
  //  ~LocalizationQueue()
  //
  //////////////////////////////////////////////////////////////////////////////
  LocalizationQueue6DOF::~LocalizationQueue6DOF()
  {
    while (list_.size() > 0)
    {
      // RemoveElement ensures that memory is properly deallocated
      RemoveElement(0);
    }
  }

  //////////////////////////////////////////////////////////////////////////////
  //
  //  initialize_queue()
  //
  //////////////////////////////////////////////////////////////////////////////
  void LocalizationQueue6DOF::initialize_queue()
  {
    // initialize location structures
    X_ = LaVectorDouble::zeros(6, 1);

    // Initialize covariance to a HIGH number to start with
    Cov_ = LaGenMatDouble::eye(6, 6);
    Cov_.scale(1e10);

    direction_ = 1.0;

    verbose_ = false;
  }

  void LocalizationQueue6DOF::set_verbose(bool verbose)
  {
    verbose_ = verbose;
  }

  //////////////////////////////////////////////////////////////////////////////
  //
  //  insert_velocity_element()
  //
  //////////////////////////////////////////////////////////////////////////////
  void LocalizationQueue6DOF::insert_velocity_element(
      const sumet_state_estimator::Velocity3DLocalizationElement &vel_elem)
  {
    // TODO(kkozak): Need to check to make sure that the timestamp makes sense
    //               before adding it to the list!!
    Velocity3DLocalizationElement* new_elem =
        new Velocity3DLocalizationElement(vel_elem);
    //    *new_elem = vel_elem;
    insert_new_element(new_elem);

    geometry_msgs::msg::TwistWithCovarianceStamped twist;
    vel_elem.get_twist(twist);
    if (twist.twist.twist.linear.x >= 0.0)
    {
      direction_ = 1.0;
    }
    else
    {
      direction_ = -1.0;
    }
  }

  //////////////////////////////////////////////////////////////////////////////
  //
  //  insert_absolute_position_element()
  //
  //////////////////////////////////////////////////////////////////////////////
  void LocalizationQueue6DOF::insert_absolute_position_element(
      const sumet_state_estimator::Absolute3DLocalizationElement &abs_elem)
  {
    if (direction_ < 0.0)
    {
      // TODO(kkozak): This prevents us from doing pitch and roll updates as
      //               well.  We may want to address this for cases where we
      //               drive reasonably long distances backwards.

      // If we're moving backwards, we'll just not update absolute position
      ROS_DEBUG("Dropped an absolute position element; going backwards.");
      return;
    }
    // TODO(kkozak): Need to check to make sure that the timestamp makes sense
    //               before adding it to the list!!
    Absolute3DLocalizationElement* new_elem =
        new Absolute3DLocalizationElement(abs_elem);
    // *new_elem = abs_elem;
    insert_new_element(new_elem);
  }

  //////////////////////////////////////////////////////////////////////////////
  //
  //  run_filter_step()
  //
  //////////////////////////////////////////////////////////////////////////////
  bool LocalizationQueue6DOF::run_filter_step(const rclcpp::Time &filter_until)
  {
    if (size() == 0)
    {
      ROS_ERROR("No data in list. Exiting.");
      return false;
    }

    LaVectorDouble X_out;
    LaVectorDouble Cov_out;
    rclcpp::Time T_actual;

    // T_actual is the actual end time stamp
    double DT = getDT(0, filter_until, T_actual);

    LocalizationElement* curElem = ReturnElement(0);

    curElem->run_update_step(X_, Cov_, X_out, Cov_out, DT, verbose_);

    X_ = X_out;
    Cov_ = Cov_out;
    timestamp_ = T_actual;

    if (size() == 1 && (ReturnElement(0)->getLocType()
        != sumet_state_estimator::Loc_Velocity))
    {
      ROS_ERROR("Preparing to crop the list to zero, Time = %f",
          ((double)filter_until.nanoseconds())/1000000000.0);
      PrintTimestampsAndTypes();
    }
    else if (size() == 1)
    {
      // This case was added to deal specifically with the situation where we
      // were filtering beyond the timestamp of our last measurement in the
      // queue (which we want to do when we're running live).  This change
      // should not affect behavior on recorded data.

      // note that returning false results in the processing of the queue being
      // stopped.
      return false;
    }
    front_crop_list(0);

    if (size() <= 0)
    {
      ROS_ERROR("cropped list to zero");
    }
    return true;
  }

  //////////////////////////////////////////////////////////////////////////////
  //
  //  process_queue()
  //
  //////////////////////////////////////////////////////////////////////////////
  bool LocalizationQueue6DOF::process_queue(rclcpp::Time T1,
      bool use_last_timestamp)
  {
    if (size() == 0)
    {
      return false;
    }

    // This check is important for the case where there is only one velocity
    // element in the list. This check is put in a separate step from the prior
    // to make it clear that it is a separate check.
    if (size() == 1)
    {
      return false;
    }

    // If use_last_timestamp is specified, the queue will be processed to the
    // time of the last element in the list rather than to the time specified
    // (T1).  This helps testing on recorded data
    if (use_last_timestamp)
    {
      T1 = ReturnTailElement()->getTimestamp();
    }

    //
    regularize_list(T1);

    if (!list_is_regularized())
    {
      ROS_ERROR("Processed an unregularized list");
    }

    // need to check these again after regularizing;
    if (size() == 0)
    {
      return false;
    }

    // This check is important for the case where there is only one velocity
    // element in the list
    if (size() == 1)
    {
      return false;
    }

    // Need to make sure that we don't crop the last velocity element out of the
    // list
    while (size() > 0 && ReturnElement(0)->getTimestamp() <= T1)
    {
      if (!run_filter_step(T1))
      {
        break;
      }

      if (ReturnElement(0)->getTimestamp() == T1
          && ReturnElement(0)->getLocType()
              == sumet_state_estimator::Loc_Velocity)
      {
        break;
      }
    }

    if (!use_last_timestamp)
    {
      timestamp_ = T1;
    }

    return true;
  }

  //////////////////////////////////////////////////////////////////////////////
  //
  // regularize_list()
  //
  //////////////////////////////////////////////////////////////////////////////
  void LocalizationQueue6DOF::regularize_list(
      const rclcpp::Time& regularize_until_time)
  {
    // The list must conform to a few (2?) basic rules:
    // 1. The first item in the list must be a velocity element
    // 2. Each absolute and/or differential element must have a velocity
    //    element with the same timestamp that immediately follows it.  To meet
    //    this requirement, velocity elements can be split.


    // Ensure that the first element is a velocity element -- this is to make
    // sure that we can carry out a prediction on each step.  Note that until
    // an absolute position measurement is received, the processed position
    // estimate will be completely wrong.
    for (int i = 0; i < size(); ++i)
    {
      // Note that the size of the list will change if we remove any elements,
      // so the size() call in the for loop statement should be used with
      // care here.  After an element is removed, the item corresponding to
      // each index, i, will change.  It is best to avoid using the 'i' index
      // at all here.

      if (ReturnElement(0)->getLocType()
          == sumet_state_estimator::Loc_Absolute
          || ReturnElement(0)->getLocType()
              == sumet_state_estimator::Loc_Differential)
      {
        RemoveElement(0);
      }
      else
      {
        break;
      }
    }

    if (size() > 0)
    {
      // Flag to check whether there's been an operation or not
      bool no_op = true;

      do
      {
        // Flag to check whether the list has been regularized
        bool isRegularized = true;
        no_op = true;

        for (int i = 0; i < size(); ++i)
        {
          if (ReturnElement(i)->getTimestamp() > regularize_until_time)
          {
            int idx = -1;
            int mode = 1;

            if (find_idx_for_timestamp(regularize_until_time, idx, mode))
            {
              // do nothing, the element is already there.
            }
            else
            {
              LastVelElem_.setTimestamp(regularize_until_time);
              insert_velocity_element(LastVelElem_);
              isRegularized = false;
              break;
            }
            break;
          }

          if (ReturnElement(i)->getLocType()
              == sumet_state_estimator::Loc_Velocity)
          {
            LastVelElem_ = *static_cast<Velocity3DLocalizationElement*>(
                ReturnElement(i));
          }

          rclcpp::Time last_time = ReturnElement(i)->getTimestamp();

          switch (element_is_regularized(i))
          {
            case ErrorNone:
              // no operation needed
              break;
            case ErrorOutOfBounds:
              ROS_ERROR("Tried to access out of bounds element in "
                        "LocalizationQueue6DOF");
              break;
            case ErrorNoVelAfterPos:
              // need to add a copy of the last velocity element with the same
              // timestamp to the end
              LastVelElem_.setTimestamp(last_time);
              insert_velocity_element(LastVelElem_);
              isRegularized = false;
              break;
            default:
              ROS_ERROR("Reached a default in a switch statement in "
                "LocalizationQueue6DOF that should not have been "
                "reached");
              break;
          }

          if (!isRegularized)
          {
            no_op = false;
            break;
          }
        }
      }
      while (!no_op);
    }
  }

  //////////////////////////////////////////////////////////////////////////////
  //
  //  find_idx_for_timestamp()
  //
  //////////////////////////////////////////////////////////////////////////////
  bool LocalizationQueue6DOF::find_idx_for_timestamp(
      const rclcpp::Time& time_to_find, int& idx, int mode)
  {
    idx = 0;
    bool element_found = false;

    for (int i = 0; i < size(); ++i)
    {
      if (ReturnElement(i)->getTimestamp() > time_to_find)
      {
        break;
      }

      if (ReturnElement(i)->getTimestamp() == time_to_find)
      {
        // if the mode is 1 (where we only count Loc_Velocity elements as
        // matches, then make sure that the element is of Loc_Velocity type.
        if (mode == 1 && ReturnElement(i)->getLocType()
            != sumet_state_estimator::Loc_Velocity)
        {
          idx = i + 1;
        }
        else
        {
          ROS_ERROR("Found timestamp element");
          idx = i;
          element_found = true;
          break;
        }
      }
      else
      {
        idx = i + 1;
      }
    }
    return element_found;
  }

  //////////////////////////////////////////////////////////////////////////////
  //
  //
  //
  //////////////////////////////////////////////////////////////////////////////
  RegError LocalizationQueue6DOF::element_is_regularized(int i)
  {
    //  ErrorNone,
    //  ErrorOutOfBounds,
    //  ErrorPosAtEnd,
    //  ErrorNoVelAfterPos

    RegError ReturnVal = ErrorNone;
    // Returns true if index is out of bounds
    if (i >= size() || i < 0)
    {
      ReturnVal = ErrorOutOfBounds;
    }
    else if (ReturnElement(i)->getLocType()
        == sumet_state_estimator::Loc_Absolute)
    {
      if (i >= size() - 1)  // if it's the last element
      {
        ReturnVal = ErrorNoVelAfterPos;
      }
      else if (ReturnElement(i + 1)->getLocType()
          != sumet_state_estimator::Loc_Velocity)
      {
        // Unless the next element has the same time:
        if (ReturnElement(i+1)->getTimestamp()
            != ReturnElement(i)->getTimestamp())
        {
          ReturnVal = ErrorNoVelAfterPos;
        }
      }
      else if (ReturnElement(i + 1)->getTimestamp()
          != ReturnElement(i)->getTimestamp())
      {
        ReturnVal = ErrorNoVelAfterPos;
      }
      else
      {
        // nothing here
      }
    }
    return ReturnVal;
  }

  //////////////////////////////////////////////////////////////////////////////
  //
  //
  //
  //////////////////////////////////////////////////////////////////////////////
  bool LocalizationQueue6DOF::list_is_regularized()
  {
    for (int i = 0; i < size(); ++i)
    {
      RegError err = element_is_regularized(i);
      if (err != ErrorNone)
      {
        if (err == ErrorNoVelAfterPos)
        {
          ROS_ERROR("List is not regularized. Element %d of %d, "
            "err: ErrorNoVelAfterPos", i, size());
        }
        else if (err == ErrorOutOfBounds)
        {
          ROS_ERROR("List is not regularized. Element %d of %d, "
            "err: ErrorOutOfBounds", i, size());
        }
        else
        {
          ROS_ERROR("List is not regularized. Element %d of %d, "
            "err: %d", i, size(), err);
        }
        return false;
      }
    }
    return true;
  }


  //////////////////////////////////////////////////////////////////////////////
  //
  //
  //
  //////////////////////////////////////////////////////////////////////////////
  double LocalizationQueue6DOF::getHeadingVariance()
  {
    return Cov_(3, 3);
  }


  //////////////////////////////////////////////////////////////////////////////
  //
  //
  //
  //////////////////////////////////////////////////////////////////////////////
  LaGenMatDouble LocalizationQueue6DOF::GetCovariance()
  {
    return Cov_;
  }

  //////////////////////////////////////////////////////////////////////////////
  //
  //
  //
  //////////////////////////////////////////////////////////////////////////////
  bool LocalizationQueue6DOF::ResetCovariance(const LaGenMatDouble& new_cov)
  {
    if (new_cov.rows() != Cov_.rows() || new_cov.cols() != Cov_.cols())
    {
      return false;
    }

    double cn = get_condition_number(new_cov);
    if (cn > 1e8)
    {
      // This is a poorly conditioned matrix, we won't use it
      return false;
    }

    Cov_ = new_cov;
    return true;
  }

  //////////////////////////////////////////////////////////////////////////////
  //
  //
  //
  //////////////////////////////////////////////////////////////////////////////
  void LocalizationQueue6DOF::getVehiclePose(
      geometry_msgs::msg::PoseWithCovarianceStamped& pose) const
  {
    pose.header.frame_id = "swri";
    pose.header.stamp = timestamp_;
    geometry_msgs::msg::Pose& p = pose.pose.pose;
    p.position.x = X_(0);
    p.position.y = X_(1);
    p.position.z = X_(2);

    tf2::Quaternion Q;
    Q.setRPY(X_(5), X_(4), X_(3));
    p.orientation.x = Q.x();
    p.orientation.y = Q.y();
    p.orientation.z = Q.z();
    p.orientation.w = Q.w();

    for (uint32_t i = 0; i < 6; i++)
    {
      for (uint32_t j = 0; j < 6; j++)
      {
        pose.pose.covariance[i * 6 + j] = Cov_(i, j);
      }
    }
  }

  //////////////////////////////////////////////////////////////////////////////
  //
  //
  //
  //////////////////////////////////////////////////////////////////////////////
  void LocalizationQueue6DOF::getVehicleTwist(
      geometry_msgs::msg::TwistWithCovarianceStamped& twist) const
  {
    LastVelElem_.get_twist(twist);
  }

  tf2::Transform getTransform(const LaVectorDouble& X)
  {
    tf2::Transform T;
    T.setOrigin(tf2::Vector3(X(0), X(1), X(2)));

    tf2::Quaternion Q;
    // Q.setEulerZYX(X(3), X(4), X(5));
    Q.setRPY(X(5), X(4), X(3));

    T.setRotation(Q);

    return T;
  }
  //////////////////////////////////////////////////////////////////////////////
  //
  //
  //
  //////////////////////////////////////////////////////////////////////////////
  bool LocalizationQueue6DOF::front_crop_list(int i)
  {
    if (i < 0 || i >= list_.size())
    {
      return false;
    }

    while (i >= 0)
    {
      RemoveElement(i);
      --i;
    }
    return true;
  }

  //////////////////////////////////////////////////////////////////////////////
  //
  // RemoveElement()
  //
  //////////////////////////////////////////////////////////////////////////////
  void LocalizationQueue6DOF::RemoveElement(int i)
  {
    // We need to delete the data in the list before removing the element from
    // the list
    delete (*(list_.ReturnElement(i)));
    list_.remove(i);
  }

  // finds the element closest (and less than) the selected time
  //////////////////////////////////////////////////////////////////////////////
  //
  // get_closest_elem()
  //
  //////////////////////////////////////////////////////////////////////////////
  int LocalizationQueue6DOF::get_closest_elem(const rclcpp::Time &T)
  {
    int min_idx = -1;
    double MinDiff = 1e20;
    for (int i = 0; i < size(); ++i)
    {
      double diff = swri::toSec(ReturnElement(i)->getTimestamp() - T);
      if (diff < 0.0)
      {
        if (abs(diff) < MinDiff)
        {
          MinDiff = abs(diff);
          min_idx = i;
        }
      }
    }
    return min_idx;
  }

  //////////////////////////////////////////////////////////////////////////////
  //
  // ReturnElement()
  //
  //////////////////////////////////////////////////////////////////////////////
  LocalizationElement* LocalizationQueue6DOF::ReturnElement(int i)
  {
    LocalizationElement** curElem = list_.ReturnElement(i);
    LocalizationElement* retElem = 0;
    if (curElem == 0)
    {
      ROS_ERROR("Tried to return data from an invalid index");
    }
    else
    {
      retElem = *curElem;
    }

    return retElem;
  }

  //////////////////////////////////////////////////////////////////////////////
  //
  // ReturnTailElement()
  //
  //////////////////////////////////////////////////////////////////////////////
  LocalizationElement* LocalizationQueue6DOF::ReturnTailElement(int i)
  {
    return (ReturnElement(size() - 1 - i));
  }

  //////////////////////////////////////////////////////////////////////////////
  //
  // getDT()
  //
  //////////////////////////////////////////////////////////////////////////////
  double LocalizationQueue6DOF::getDT(int idx, const rclcpp::Time& Tmax,
      rclcpp::Time& T_actual)
  {
    double dt = 0.0;
    if (size() > idx + 1)
    {
      rclcpp::Time nextTime = ReturnElement(idx + 1)->getTimestamp();
      rclcpp::Time curTime = ReturnElement(idx)->getTimestamp();
      if (nextTime < Tmax)
      {
        T_actual = nextTime;
        dt = swri::toSec(nextTime - curTime);
      }
      else
      {
        T_actual = Tmax;
        dt = swri::toSec(Tmax - curTime);
      }
    }
    else if (size() > idx)
    {
      rclcpp::Time curTime = ReturnElement(idx)->getTimestamp();
      dt = swri::toSec(Tmax - curTime);
      T_actual = ReturnElement(idx)->getTimestamp();
    }
    else
    {
      T_actual = swri::TIME_MAX;
    }
    return dt;
  }

  //////////////////////////////////////////////////////////////////////////////
  //
  // size()
  //
  //////////////////////////////////////////////////////////////////////////////
  int LocalizationQueue6DOF::size()
  {
    return (list_.size());
  }

  //////////////////////////////////////////////////////////////////////////////
  //
  //  insert_new_element()
  //
  //////////////////////////////////////////////////////////////////////////////
  void LocalizationQueue6DOF::insert_new_element(LocalizationElement* new_elem)
  {
    if (size() <= 0)
    {
      if (new_elem->getLocType() == sumet_state_estimator::Loc_Absolute)
      {
        // Don't add an absolute localization element to the first spot in the
        // list.  And the new_elem must be deleted here if it is not added to
        // the list.
        delete new_elem;
        return;
      }
      else
      {
        list_.addCopy(new_elem);
        return;
      }
    }

    // insert in order
    bool insert_success = false;
    for (int i = 0; i < size(); ++i)
    {
      // note the definition of the '<' operator for LocalizationElement -->
      // it is important to use this operator rather than just comparing the
      // timestamps because the comparison between different types gives
      // different results for equalities
      if ((*new_elem) < (*(ReturnElement(i))))
      {
        if (i == 0)
        {
          // Don't add old data, need to delete the element before returning
          delete new_elem;
          return;
        }
        else
        {
          list_.insertCopyAt(new_elem, i);
        }
        // it is essential to break here because we change the size of the list
        insert_success = true;
        break;
      }
    }

    // if we haven't inserted the element, that means that it should be added to
    // the end of the list
    if (!insert_success)
    {
      list_.addCopy(new_elem);
    }
  }

  //////////////////////////////////////////////////////////////////////////////
  //
  // PrintTimestampsAndTypes()
  //
  //////////////////////////////////////////////////////////////////////////////
  void LocalizationQueue6DOF::PrintTimestampsAndTypes()
  {
    if (size() > 0)
    {
      ROS_ERROR("=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-");
      for (int i = 0; i < size(); ++i)
      {
        uint64_t time = ReturnElement(i)->getTimestamp().nanoseconds();
        uint64_t time_sec = time / 1000000000ull;
        uint64_t time_nsec = time - time_sec*1000000000ull;
        // Doubles can't fully represent the timestamp to the nano second, so
        // we'll print it by breaking down the full nanosecond representation
        // (because we can't get the native representation of seconds +
        // nanoseconds out directly)
        ROS_ERROR("Item number %d: Timestamp: %lu.%lu, type = %d", i,
            time_sec, time_nsec,
            ReturnElement(i)->getLocType());
      }
      ROS_ERROR("=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-");
    }
  }
}
