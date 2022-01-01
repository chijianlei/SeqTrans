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
 * localization_tools.cpp
 *
 *  Created on: Jun 28, 2011
 *      Author: kkozak
 */
#include <sumet_state_estimator/localization_tools.h>

#include <algorithm>
#include <vector>

namespace sumet_state_estimator
{
  void PredictPlanarForDT(double dt,
                          const double theta,
                          const LaVectorDouble &V_in,
                          LaVectorDouble &DX_out)
  {
    double v = V_in(0);
    double w = V_in(1);

    double &dx = DX_out(0);
    double &dy = DX_out(1);
    double &dtheta = DX_out(2);

  // The constant curvature predictor has significant round-off error issues for
  // small values of angular velocity.  Setting a min value for which we'll use
  // the constant curvature predictor, rather than the straight line predictor
  // helps to control the error.  Values around 5.0e-9 as a minimum seem to work
  // well.
    const double angular_velocity_min_for_prediction = 5.0e-9;
    if (std::abs(w) < angular_velocity_min_for_prediction)
    {
      dx = v * cos(theta) * dt;
      dy = v * sin(theta) * dt;
    }
    else
    {
      dx = -v / w * (sin(theta) - sin(theta + w * dt));
      dy = v / w * (cos(theta) - cos(theta + w * dt));
    }
    dtheta = w * dt;
  }


  void gen_G_and_V_6DOF(const tf2::Vector3& x,
                        const tf2::Vector3& rpy,
                        const tf2::Vector3& vel_in,
                        const tf2::Vector3& w_in,
                        double dt,
                        LaGenMatDouble &G,
                        LaGenMatDouble &V)
  {
    // TODO(kkozak): This can be made a little more rigorous (but it may not be
    // worth it). We basically assume that the planar and non-planar motions are
    // decoupled for the generation of these matrices.  This will probably only
    // be an issue for high pitch or roll values.

    G = LaGenMatDouble::eye(6, 6);

    V = LaGenMatDouble::zeros(6, 4);

    double v = vel_in.x();
    double w = w_in.z();
    double theta = rpy.z();
    double pitch = rpy.y();

    const double angular_velocity_min_for_prediction = 5.0e-9;
    if (std::abs(w) < angular_velocity_min_for_prediction)
    {
      G(0, 3) = -v * sin(theta) * dt;
      G(1, 3) = v * cos(theta) * dt;
      // TODO(kkozak): Leaving the following term in eventually results in pitch
      //               being coupled with other states in the update step (which
      //               can result in odd behavior)
      // G(2, 4) = v * cos(pitch) * dt;

      V(0, 0) = cos(theta) * dt;
      V(1, 0) = sin(theta) * dt;
      V(2, 0) = sin(pitch) * dt;
      V(3, 1) = dt;
      V(4, 2) = dt;
      V(5, 3) = dt;
    }
    else
    {
      G(0, 3) = -(v / w) * cos(theta) + (v / w) * cos(theta + w * dt);
      G(1, 3) = -v / w * sin(theta) + v / w * sin(theta + w * dt);
      // TODO(kkozak): Leaving the following term in eventually results in pitch
      //               being coupled with other states in the update step (which
      //               can result in odd behavior)
      // G(2, 4) = v * cos(pitch) * dt;

      V(0, 0) = (-sin(theta) + sin(theta + w * dt)) / w;
      V(0, 1) = v * (sin(theta) - sin(theta + w * dt)) / pow(w, 2) + v * cos(
          theta + w * dt) * dt / w;

      V(1, 0) = (cos(theta) - cos(theta + w * dt)) / w;
      V(1, 1) = -v * (cos(theta) - cos(theta + w * dt)) / pow(w, 2) + v * sin(
          theta + w * dt) * dt / w;

      V(2, 0) = sin(pitch) * dt;
      V(3, 1) = dt;
      V(4, 2) = dt;
      V(5, 3) = dt;
    }

    return;
  }

  void Predict6DofForDT(double dt,
                        const tf2::Quaternion& Q_0,
                        const tf2::Vector3& X_0,
                        const tf2::Vector3& v_in,
                        const tf2::Vector3& w_in,
                        tf2::Vector3& X_out,
                        tf2::Quaternion& Q_out)
  {
    tf2::Vector3 DX;
    PredictDiff6DofForDT(dt, Q_0, v_in, w_in, DX, Q_out);
    X_out = DX + X_0;
  }

  void PredictDiff6DofForDT(double dt,
                            const tf2::Quaternion& Q_0,
                            const tf2::Vector3& v_in,
                            const tf2::Vector3& w_in,
                            tf2::Vector3& DX_out,
                            tf2::Quaternion& Q_out)
  {
    // First, use the PredictPlanarForDT function to predict the motion of the
    // car in the vehicle frame (use the longitudinal velocity and the yaw rate)
    LaVectorDouble V_planar(2, 1);
    V_planar(0) = v_in.x();
    V_planar(1) = w_in.z();
    LaVectorDouble DX_planar(3, 1);
    PredictPlanarForDT(dt, 0.0, V_planar, DX_planar);

    // TODO(kkozak): Here we'll throw on a prediction using lateral
    // and vertical velocities. It is assumed that these are generally
    // zero, but we'll allow for the option to have non-zero values.

    // Second, rotate that prediction into the initial vehicle frame using Q_0
    tf2::Transform DX;
    DX.setOrigin(tf2::Vector3(DX_planar(0), DX_planar(1), 0.0));

    tf2::Transform Transform;
    Transform.setIdentity();
    Transform.setRotation(Q_0);

    // TODO(kkozak): check multiplication order
    DX_out = (Transform * DX).getOrigin();

    // Third, predict the new attitude by integrating the angular velocity
    // vector (sort of)
    // TODO(kkozak): May need to compare heading accuracy between this
    // method and just using the yaw rate sensor as the heading
    // estimator (i.e. need to see whether faulty pitch estimates
    // throw pollute the accuracy
    Q_out = integrate_angular_velocity(dt, w_in, Q_0);
  }


  tf2::Quaternion integrate_angular_velocity(double dt,
                                            const tf2::Vector3& w_in,
                                            const tf2::Quaternion& Q_0)
  {
    double w_mag = w_in.length();

    tf2::Vector3 w(0.0, 0.0, 1.0);

    // If the angular velocity is near zero, we'll just force the axis to be
    // vertical (wz = 1)
    if (w_mag > 1.0e-9)
    {
      w = w_in.normalized();
    }
    double dtheta = dt * w_mag;
    // Create the current quaternion transformation for the small rotation based
    // upon the angular velocity vector
    tf2::Quaternion Qtransform(w.x() * std::sin(dtheta / 2), w.y() * std::sin(
        dtheta / 2), w.z() * std::sin(dtheta / 2), std::cos(dtheta / 2));
    Qtransform.normalize();

    return (Q_0 * Qtransform).normalize();
  }

  double get_cov_by_idx_3x3(const std::array<double, 9>& cov_in,
                            int32_t row,
                            int32_t col)
  {
    return (cov_in[row * 3 + col]);
  }

  double get_cov_by_idx_6x6(const std::array<double, 36>& cov_in,
                            int32_t row,
                            int32_t col)
  {
    return (cov_in[row * 6 + col]);
  }

  void set_cov_by_idx_3x3(std::array<double, 9>& cov_in,  // NOLINT
                          int32_t row,
                          int32_t col,
                          double val)
  {
    cov_in[row * 3 + col] = val;
  }

  void set_cov_by_idx_6x6(std::array<double, 36>& cov_in,  // NOLINT
                          int32_t row,
                          int32_t col,
                          double val)
  {
    cov_in[row * 6 + col] = val;
  }


  double get_condition_number(const LaGenMatDouble& mat_in)
  {
    LaGenMatDouble A(mat_in);
    LaVectorDouble Sigma =
        LaVectorDouble::zeros(std::min(A.rows(), A.cols()), 1);
    LaGenMatDouble U = LaGenMatDouble::zeros(A.rows(), A.rows());
    LaGenMatDouble Vt = LaGenMatDouble::zeros(A.cols(), A.cols());

    LaSVD_IP(A, Sigma, U, Vt);
    double max1 = Sigma(0);
    double min1 = Sigma(0);
    for (int32_t i = 1; i < Sigma.size(); ++i)
    {
      if (std::abs(Sigma(i)) > max1)
      {
        max1 = std::abs(Sigma(i));
      }
      else if (std::abs(Sigma(i)) < min1)
      {
        min1 = std::abs(Sigma(i));
      }
    }
    return max1 / min1;
  }
}
