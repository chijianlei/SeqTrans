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
 * DerivativeEstimator.cpp
 *
 *  Created on: Jan 26, 2012
 *      Author: kkozak
 */
#include <sumet_state_estimator/DerivativeEstimator.h>

#include <algorithm>

namespace sumet_state_estimator
{
  DerivativeEstimator::DerivativeEstimator()
  {
    data.set_capacity(1000);
  }

  void DerivativeEstimator::add_new_data(const ros::Time& time1,
                                         double curData)
  {
    sumet_state_estimator::StampedDoubleData data1;
    data1.header.stamp = time1;
    data1.data = curData;
    data.push_back(data1);
  }

  double DerivativeEstimator::get_derivative(int N)
  {
    N = std::min(static_cast<int>(data.size()), N);
    if (N <= 1)
    {
      return 0.0;
    }
    sumet_state_estimator::StampedDoubleData dat2 = data.back();
    sumet_state_estimator::StampedDoubleData dat1 = data[data.size() - N];
    double dt = (dat2.header.stamp - dat1.header.stamp).toSec();
    if (dt <= 1.0e-15)
    {
      // Check to make sure that dt is valid
      return 0.0;
    }

    double dx = dat2.data - dat1.data;
    return dx/dt;
  }
}

