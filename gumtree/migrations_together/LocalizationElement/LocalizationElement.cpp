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
 * LocalizationElement.cpp
 *
 *  Created on: Jul 29, 2011
 *      Author: kkozak
 */

#include <sumet_state_estimator/LocalizationElement.h>

namespace sumet_state_estimator
{
  //////////////////////////////////////////////////////////////////////////////
  //
  //  print_mat()
  //
  //////////////////////////////////////////////////////////////////////////////
  void print_mat(const LaGenMatDouble& M, int idx1)
  {
    fprintf(stderr, "Matrix %d= \n", idx1);
    for (int i = 0; i < M.rows(); ++i)
    {
      fprintf(stderr, "[");
      for (int j = 0; j < M.cols(); ++j)
      {
        fprintf(stderr, "%f", M(i, j));
        if (j < M.cols() - 1)
        {
          fprintf(stderr, ",");
        }
      }
      fprintf(stderr, "]\n");
    }
  }

  //////////////////////////////////////////////////////////////////////////////
  //
  //  LocalizationElement()
  //
  //////////////////////////////////////////////////////////////////////////////
  LocalizationElement::LocalizationElement()
  {
    initialize();
  }

  //////////////////////////////////////////////////////////////////////////////
  //
  //  LocalizationElement()
  //
  //////////////////////////////////////////////////////////////////////////////
  LocalizationElement::LocalizationElement(
      sumet_state_estimator::LocType type)
  {
    initialize(type);
  }

  //////////////////////////////////////////////////////////////////////////////
  //
  //  operator<()
  //
  //////////////////////////////////////////////////////////////////////////////
  bool LocalizationElement::operator<(const LocalizationElement & cmp)
  {
    // operator compares timestamps, if the object's timestamp is less than that
    // for cmp, the the function returns true.  If timestamps are the same then
    // the following behavior should be generated: T1 == T2 -> 1. Loc_Velocity
    // types > Loc_Absolute types > Loc_Differential types. In principle, the
    // differential and absolute order can be swapped, but this hasn't been
    // tested.  The Loc_Velocity types MUST come last for the LocalizationQueue
    // to work properly.

    // TODO(kkozak): for use_original_logic = true, the behavior in
    // the localization queue is the same as it was originally, but it
    // will not necessarily work for Loc_Differential cases... that
    // case still needs to be defined
    const bool use_original_logic = true;

    if (use_original_logic)
    {
      if (this->getTimestamp() < cmp.getTimestamp())
      {
        // this is the normal less than case
        return true;
      }
      else if (this->getTimestamp() == cmp.getTimestamp() && LocalizerType
          == sumet_state_estimator::Loc_Absolute)
      {
        // If the timesteps are are equal then velocity must come last
        return true;
      }
      else
      {
        return false;
      }
    }
    else
    {
      if (this->getTimestamp() < cmp.getTimestamp())
      {
        // this is the normal less than case
        return true;
      }
      else if (this->getTimestamp() == cmp.getTimestamp() && LocalizerType
          == sumet_state_estimator::Loc_Differential)
      {
        // if the timestamps are equal, then we want to put Loc_Differential
        // types first
        return true;
      }
      else if (this->getTimestamp() == cmp.getTimestamp() && LocalizerType
          == sumet_state_estimator::Loc_Absolute && cmp.getLocType()
          == sumet_state_estimator::Loc_Velocity)
      {
        // This puts Loc_Absolute types after differential, but before velocity
        // types.
        return true;
      }
      else
      {
        return false;
      }
    }
  }

  //////////////////////////////////////////////////////////////////////////////
  //
  //  split_element()
  //
  //////////////////////////////////////////////////////////////////////////////
  bool LocalizationElement::split_element(ros::Time time_to_split,
      LocalizationElement & returnElem)
  {
    if (time_to_split < getTimestamp())
    {
      return false;
    }

    returnElem = *this;
    returnElem.setTimestamp(time_to_split);
    //  returnElem.setSplit();
    return true;
  }

  //////////////////////////////////////////////////////////////////////////////
  //
  //  getTimestamp()
  //
  //////////////////////////////////////////////////////////////////////////////
  ros::Time LocalizationElement::getTimestamp() const
  {
    return timestamp;
  }

  //////////////////////////////////////////////////////////////////////////////
  //
  //  setTimestamp()
  //
  //////////////////////////////////////////////////////////////////////////////
  void LocalizationElement::setTimestamp(const ros::Time &T1)
  {
    timestamp = T1;
  }

  //////////////////////////////////////////////////////////////////////////////
  //
  //  getLocType()
  //
  //////////////////////////////////////////////////////////////////////////////
  sumet_state_estimator::LocType LocalizationElement::getLocType() const
  {
    return LocalizerType;
  }

  //////////////////////////////////////////////////////////////////////////////
  //
  //  setLocType()
  //
  //////////////////////////////////////////////////////////////////////////////
  void LocalizationElement::setLocType(sumet_state_estimator::LocType type)
  {
    LocalizerType = type;
  }

  //////////////////////////////////////////////////////////////////////////////
  //
  //  initialize()
  //
  //////////////////////////////////////////////////////////////////////////////
  void LocalizationElement::initialize(sumet_state_estimator::LocType type)
  {
    LocalizerType = type;
    // isSplit = false;
  }
}
