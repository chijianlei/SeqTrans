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
 * BiasCalcClass.cpp
 *
 *  Created on: Jan 26, 2012
 *      Author: kkozak
 */

#include <sumet_state_estimator/BiasCalcClass.h>

namespace sumet_state_estimator
{
  BiasCalcClass::BiasCalcClass()
  {
    start_acquisition_time_ = ros::Time::now();
    is_active_ = false;
    min_bias_time_ = 10.0;
    max_bias_time_ = 60.0;
    max_acceptable_delay_ = 0.5;
    current_bias_ = 0.0;
    bias_is_valid_ = false;
    bias_validity_duration_ = 10.0 * 60.0;
    last_data_time_ = ros::TIME_MIN;
    bias_computation_time_ = ros::TIME_MIN;
    is_initialized_ = false;
    is_recalculating_ = false;
    NumberOfSamplesBetweenAverages_ = 100;
    cur_num_samples_ = 0;

    initialize(1000);
  }

  void BiasCalcClass::initialize(int NumSamplesToAverage)
  {
    NumberOfSamplesToAverage_ = NumSamplesToAverage;
    maf_.initialize(NumSamplesToAverage);
  }

  void BiasCalcClass::set_averaging_period(int NumSamplesBetweenAverages)
  {
    NumberOfSamplesBetweenAverages_ = NumSamplesBetweenAverages;
  }

  double BiasCalcClass::get_current_bias()
  {
    return current_bias_;
  }

  ros::Duration BiasCalcClass::get_bias_age()
  {
    ros::Time curTime = ros::Time::now();
    return (curTime - bias_computation_time_);
  }

  bool BiasCalcClass::get_is_bias_current()
  {
    return (get_bias_age().toSec() < bias_validity_duration_);
  }

  void BiasCalcClass::set_bias_validity_duration(double duration)
  {
    bias_validity_duration_ = duration;
  }

  bool BiasCalcClass::load_new_data(double data, bool force)
  {
    ros::Time curTime = ros::Time::now();
    // Time elapsed from the last sample added
    double cur_duration = (curTime - last_data_time_).toSec();
    last_data_time_ = curTime;

    // Check to see whether the buffer should be reset (because of a long delay)
    if (!is_active_ || cur_duration > max_acceptable_delay_)
    {
      // We've received the first sample, or the current data is new
      // clear the buffer
      maf_.initialize(NumberOfSamplesToAverage_);

      // reset the bias calculation
      is_active_ = true;
      is_initialized_ = false;
      start_acquisition_time_ = curTime;
      cur_num_samples_ = 0;
      maf_.fastAppendElement(data);
    }
    else
    {
      bool init_state = is_initialized_;
      is_initialized_ = maf_.fastAppendElement(data);
      if (is_initialized_)
      {
        if (!init_state || (cur_num_samples_++
            % NumberOfSamplesBetweenAverages_) == 0 || force)
        {
          // depending upon the size of the buffer, the averaging calculation
          // can be somewhat expensive, so only calculate the average if we have
          // just initialized (i.e. filled up) the filter, or if we've waited
          // for NumberOfSamplesBetweenAverages_ samples.
          current_bias_ = maf_.getAverage();
          bias_computation_time_ = curTime;
          // ROS_ERROR("Current bias = %15.10f",current_bias_);
        }
      }
    }
    return is_initialized_;
  }
}
