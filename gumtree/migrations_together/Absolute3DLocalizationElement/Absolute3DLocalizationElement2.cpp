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
 * Absolute3DLocalizationElement.cpp
 *
 *  Created on: Feb 17, 2012
 *      Author: kkozak
 */

#include <sumet_state_estimator/Absolute3DLocalizationElement.h>

#include <algorithm>

#include <swri_roscpp/logging.h>
#include <swri_roscpp/time.h>

namespace sumet_state_estimator
{
  //////////////////////////////////////////////////////////////////////////////
  //
  //  Absolute3DLocalizationElement()
  //
  //////////////////////////////////////////////////////////////////////////////
  Absolute3DLocalizationElement::Absolute3DLocalizationElement()
  {
    Cov_ = LaGenMatDouble::eye(6, 6);
    Cov_ *= sumet_util::_large_variance;
    T_ = tf2::Transform::getIdentity();
    X_ = LaVectorDouble::zeros(6, 1);
    this->setLocType(sumet_state_estimator::Loc_Absolute);
  }

  //////////////////////////////////////////////////////////////////////////////
  //
  //  Absolute3DLocalizationElement()
  //
  //////////////////////////////////////////////////////////////////////////////
  Absolute3DLocalizationElement::Absolute3DLocalizationElement(
      const Absolute3DLocalizationElement& elem) :
    LocalizationElement(elem)
  {
    Cov_.copy(elem.Cov_);
    X_.copy(elem.X_);
    T_ = elem.T_;
  }

  //////////////////////////////////////////////////////////////////////////////
  //
  //  run_update_step()
  //
  //////////////////////////////////////////////////////////////////////////////
  void Absolute3DLocalizationElement::run_update_step(
      const tf2::Transform& T_cur,
      const LaGenMatDouble& Cov_cur,
      tf2::Transform& T_out,
      LaGenMatDouble& Cov_out,
      double DT,
      bool verbose)
  {
    // T_cur is the current position position and orientation estimate
    // based upon the previous update

    // Matlab steps for EKF correction step
    //
    // Hk = eye(6);                               // Step 1
    // yk = XGPScur - Hk*XEstTemp(:);             // Step 2
    // Sk = Hk*Sigma1*Hk' + M2;                   // Step 3
    // Kk = Sigma1*Hk'*inv(Sk);                   // Step 4
    // XestNew1 = XEstTemp + Kk*yk;               // Step 5
    // Sigma1temp = (eye(6)-Kk*Hk)*Sigma1;        // Step 6

    ROS_DEBUG("Running absolute update step \n Values: (%f, %f, %f) (%f, %f, %f) \n Variances: (%f, %f, %f) (%f, %f, %f)",
          X_(0), X_(1), X_(2),
          X_(3), X_(4), X_(5),
          Cov_(0,0), Cov_(1,1), Cov_(2,2),
          Cov_(3,3), Cov_(4,4), Cov_(5,5));

    // State vector: X = {x; y; z; yaw; pitch; roll};
    const int32_t num_states = 6;

    LaVectorDouble X_cur = LaVectorDouble::zeros(num_states, 1);
    tf2::Vector3 x_vec = T_cur.getOrigin();

    X_cur(0) = x_vec.x();
    X_cur(1) = x_vec.y();
    X_cur(2) = x_vec.z();

    // Aside from yaw, angles will be small most of the time, so we'll use the
    // Euler angles as the states.  If this presents problems, we can use a
    // quaternion instead.
    // in order: 5: roll, 4: pitch, 3: yaw
    T_cur.getBasis().getRPY(X_cur(5), X_cur(4), X_cur(3));

    // make sure that pitch and roll are between -pi and pi:
    X_cur(4) = sumet_util::MathUtil::FixAngleMinusPitoPi(X_cur(4));
    X_cur(5) = sumet_util::MathUtil::FixAngleMinusPitoPi(X_cur(5));

    // Step 0
    // make sure that the angle difference makes sense:
    // LaVectorDouble X_cur_temp = LaVectorDouble::zeros(num_states, 1);
    LaVectorDouble X_cur_temp = X_cur;

    // Pitch and roll should stay confined to relatively small angles, but yaw
    // can go beyond 360 degrees, and so the values must be "unwrapped" if they
    // cross over the 0-2pi threshold to bad filter results.

    // alias the values so that it's clear what we're operating on
    double& yaw_cur = X_cur_temp(3);
    double& yaw_meas = X_(3);
    yaw_meas = sumet_util::MathUtil::WrapAngle(yaw_cur, yaw_meas);

    // Step 1
    // This step is not necessary here --> we can for all practical purposes
    // measure all the states directly, but we may want to revise this later on

    LaGenMatDouble Hk = getHk();

    // Step 2
    LaVectorDouble yk;
    yk = Hk * X_ - Hk * X_cur_temp;

    // Step 3
    LaGenMatDouble Stemp = LaGenMatDouble::zeros(Hk.rows(), Cov_cur.cols());
    Blas_Mat_Mat_Mult(Hk, Cov_cur, Stemp);  // Stemp = Hk*Cov_cur

    LaGenMatDouble Sk = LaGenMatDouble::zeros(Stemp.rows(), Hk.rows());
    Blas_Mat_Mat_Trans_Mult(Stemp, Hk, Sk);  // Sk = Cov_cur*Hk'


    LaGenMatDouble Cov_shrunk_temp = LaGenMatDouble::zeros(Hk.rows(),
        Cov_.cols());
    Blas_Mat_Mat_Mult(Hk, Cov_, Cov_shrunk_temp);  // Cov_shrunk_temp = Hk*Cov_
    LaGenMatDouble Cov_shrunk = LaGenMatDouble::zeros(Hk.rows(), Hk.rows());

    // Cov_shrunk = Cov_shrunk_temp*Hk'
    Blas_Mat_Mat_Trans_Mult(Cov_shrunk_temp, Hk, Cov_shrunk);

    Sk = Sk + Cov_shrunk;

    // Step 4
    LaGenMatDouble Kk_temp = LaGenMatDouble::zeros(Cov_cur.rows(), Hk.rows());
    Blas_Mat_Mat_Trans_Mult(Cov_cur, Hk, Kk_temp);  // Kk_temp = Cov_cur*Hk' =
                                                    // Sigma1*Hk'

    // Now solve  Kk*Sk = Kk_temp; ==> Sk' * Kk' = Kk_temp'
    LaGenMatDouble Kk = LaGenMatDouble::zeros(Kk_temp.rows(), Kk_temp.cols());

    mat_transpose(Kk);
    mat_transpose(Kk_temp);
    mat_transpose(Sk);

    if (verbose)
    {
      double cond1 = get_condition_number(Sk);
      if (cond1 > 1e8)
      {
        ROS_ERROR("Matrix conditioning is poor.  Condition number of Sk = %g",
                  cond1);
      }
    }
    LaLinearSolve(Sk, Kk, Kk_temp);  // This is equivalent to Kk' =
                                     // Sk'\Kk_temp' in Matlab
    mat_transpose(Kk);

    // Step 5
    LaVectorDouble XnewTemp = LaVectorDouble::zeros(num_states, 1);
    Blas_Mat_Vec_Mult(Kk, yk, XnewTemp);

    LaVectorDouble X_out;
    X_out = X_cur_temp + XnewTemp;

    // Step 6
    LaGenMatDouble TempOut = LaGenMatDouble::eye(num_states, num_states);
    Blas_Mat_Mat_Mult(Kk, Hk, TempOut, -1.0, 1.0);

    Cov_out = LaGenMatDouble::zeros(num_states, num_states);
    Blas_Mat_Mat_Mult(TempOut, Cov_cur, Cov_out);

    X_out(3) = sumet_util::MathUtil::FixAngle0to2Pi(X_out(3));

    tf2::Vector3 Xvec(X_out(0), X_out(1), X_out(2));
    tf2::Quaternion Q;
    Q.setRPY(X_out(5), X_out(4), X_out(3));
    T_out = tf2::Transform(Q, Xvec);
  }

  //////////////////////////////////////////////////////////////////////////////
  //
  //  mat_transpose()
  //
  //////////////////////////////////////////////////////////////////////////////
  void Absolute3DLocalizationElement::mat_transpose(LaGenMatDouble& M)  // NOLINT
  {
    if (M.rows() != M.cols())
    {
      LaGenMatDouble Temp = LaGenMatDouble::zeros(M.cols(), M.rows());
      for (int i = 0; i < M.rows(); i++)
      {
        for (int j = 0; j < M.cols(); j++)
        {
          Temp(j, i) = M(i, j);
        }
      }
      M = Temp;
    }
    else
    {
      for (int i = 0; i < M.rows(); i++)
      {
        for (int j = i + 1; j < M.rows(); j++)
        {
          double temp = M(i, j);
          M(i, j) = M(j, i);
          M(j, i) = temp;
        }
      }
    }
  }

  //////////////////////////////////////////////////////////////////////////////
  //
  //  run_update_step()
  //
  //////////////////////////////////////////////////////////////////////////////
  void Absolute3DLocalizationElement::run_update_step(
      const LaVectorDouble& X_cur,
      const LaGenMatDouble& Cov_cur,
      LaVectorDouble& X_out,
      LaGenMatDouble& Cov_out,
      double DT,
      bool verbose)
  {
    X_out = LaVectorDouble::zeros(6, 1);
    Cov_out = LaVectorDouble::eye(6, 6);
    // TODO(kkozak): After writing the other "run_update_step" I realized that
    // we needed to maintain the same interface, so that's being done with this
    // somewhat inefficient shuffling of variables
    tf2::Vector3 X(X_cur(0), X_cur(1), X_cur(2));
    tf2::Quaternion Q;

    Q.setRPY(X_cur(5), X_cur(4), X_cur(3));

    tf2::Transform T(Q, X);

    // Create T_out for storage of transform results
    tf2::Transform T_out;

    // Call other run_update_step function  Cov_out gets filled directly here
    run_update_step(T, Cov_cur, T_out, Cov_out, DT, verbose);

    // Fill in the output values
    X = T_out.getOrigin();

    X_out(0) = X.x();
    X_out(1) = X.y();
    X_out(2) = X.z();

    T_out.getBasis().getRPY(X_out(5), X_out(4), X_out(3));
  }

  //////////////////////////////////////////////////////////////////////////////
  //
  //  load_pos_and_cov_data()
  //
  //////////////////////////////////////////////////////////////////////////////
  void Absolute3DLocalizationElement::load_pos_and_cov_data(
      const LaVectorDouble& X_in, const LaGenMatDouble& Cov_in)
  {
    X_ = X_in;
    Cov_ = Cov_in;
  }

  //////////////////////////////////////////////////////////////////////////////
  //
  //  load_pos_and_cov_data()
  //
  //////////////////////////////////////////////////////////////////////////////
  void Absolute3DLocalizationElement::load_pos_and_cov_data(
      const tf2::Transform& T_in, const LaGenMatDouble& Cov_in)
  {
    tf2::Vector3 Xtemp = T_in.getOrigin();

    double yaw = 0.0;
    double pitch = 0.0;
    double roll = 0.0;
    T_in.getBasis().getRPY(roll, pitch, yaw);
    X_(0) = Xtemp.x();
    X_(1) = Xtemp.y();
    X_(2) = Xtemp.z();
    X_(3) = yaw;
    X_(4) = pitch;
    X_(5) = roll;
    Cov_ = Cov_in;
  }

  //////////////////////////////////////////////////////////////////////////////
  //
  //  load_pose_data_from_msg()
  //
  //////////////////////////////////////////////////////////////////////////////
  void Absolute3DLocalizationElement::load_pose_data_from_msg(
      const geometry_msgs::msg::PoseWithCovarianceStamped& msg)
  {
    this->setTimestamp(msg.header.stamp);

    const geometry_msgs::msg::Point& x = msg.pose.pose.position;

    X_(0) = x.x;
    X_(1) = x.y;
    X_(2) = x.z;

    const geometry_msgs::msg::Quaternion& Qm = msg.pose.pose.orientation;
    tf2::Transform transform(tf2::Quaternion(Qm.x, Qm.y, Qm.z, Qm.w));
    transform.getBasis().getRPY(X_(5), X_(4), X_(3));

    for (int i = 0; i < 6; i++)
    {
      for (int j = 0; j < 6; j++)
      {
        Cov_(i, j) = msg.pose.covariance[i * 6 + j];
      }
    }
    ROS_DEBUG("Abs Position Elem Loaded Pose Data From Msg. \nValues: (%f, %f, %f) (%f, %f, %f) \nVariances: (%f, %f, %f) (%f, %f, %f)",
          X_(0), X_(1), X_(2),
          X_(3), X_(4), X_(5),
          Cov_(0,0), Cov_(1,1), Cov_(2,2),
          Cov_(3,3), Cov_(4,4), Cov_(5,5));
  }

  //////////////////////////////////////////////////////////////////////////////
  //
  //  report_data()
  //
  //////////////////////////////////////////////////////////////////////////////
  void Absolute3DLocalizationElement::report_data() const
  {
    tf2::Vector3 Xtemp = T_.getOrigin();

    double yaw = 0.0;
    double pitch = 0.0;
    double roll = 0.0;
    T_.getBasis().getRPY(roll, pitch, yaw);

    ROS_ERROR("Time: %f, Type: Absolute, x = %f, y = %f, z = %f, yaw = %f, "
        "pitch = %f, roll = %f",
        swri::toSec(this->getTimestamp()),
        Xtemp.x(),
        Xtemp.y(),
        Xtemp.z(),
        yaw,
        pitch,
        roll);
  }

  //////////////////////////////////////////////////////////////////////////////
  //
  //  set_position_data()
  //
  //////////////////////////////////////////////////////////////////////////////
  void Absolute3DLocalizationElement::set_position_data(
    const tf2::Transform& T_in)
  {
    T_ = T_in;
    const int32_t num_states = 6;
    LaVectorDouble X_cur = LaVectorDouble::zeros(num_states, 1);
    tf2::Vector3 x_vec = T_in.getOrigin();

    X_(0) = x_vec.x();
    X_(1) = x_vec.y();
    X_(2) = x_vec.z();

    // Aside from yaw, angles will be small most of the time, so we'll
    // use the Euler angles as the states.  If this presents problems,
    // we can use a quaternion instead.

    // in order: 5: roll, 4: pitch, 3: yaw
    T_in.getBasis().getRPY(X_(5), X_(4), X_(3));

    // make sure that pitch and roll are between -pi and pi:
    X_(3) = sumet_util::MathUtil::FixAngle0to2Pi(X_(3));
    X_(4) = sumet_util::MathUtil::FixAngleMinusPitoPi(X_(4));
    X_(5) = sumet_util::MathUtil::FixAngleMinusPitoPi(X_(5));
  }

  //////////////////////////////////////////////////////////////////////////////
  //
  //  getHk()
  //
  //////////////////////////////////////////////////////////////////////////////
  LaGenMatDouble Absolute3DLocalizationElement::getHk()
  {
    LaGenMatDouble diag = Cov_.diag();
    int num_vals = 0;
    for (int i = 0; i < diag.rows(); i++)
    {
      if (diag(i, 0) < 1e12)
      {
        num_vals++;
      }
    }

    LaGenMatDouble out = LaGenMatDouble::zeros(num_vals, diag.rows());
    int idx = 0;
    for (int i = 0; i < diag.rows(); i++)
    {
      if (diag(i, 0) < 1e12)
      {
        out(idx, i) = 1.0;
        idx++;
      }
    }

    return out;
  }
}
