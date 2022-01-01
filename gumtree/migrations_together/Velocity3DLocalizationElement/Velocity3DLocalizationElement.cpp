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
 * Velocity3DLocalizationElement.cpp
 *
 *  Created on: Mar 21, 2012
 *      Author: kkozak
 */

#include <sumet_state_estimator/Velocity3DLocalizationElement.h>

#include <algorithm>

namespace sumet_state_estimator
{
  LaGenMatDouble Velocity3DLocalizationElement::four_by_six_update_matrix_ = LaGenMatDouble::zeros(4, 6);
  bool Velocity3DLocalizationElement::update_matrix_initialized_ = false;

  //////////////////////////////////////////////////////////////////////////////
  //
  // VelocityLocalizationElement()
  //
  //////////////////////////////////////////////////////////////////////////////
  Velocity3DLocalizationElement::Velocity3DLocalizationElement()
  {
    V_.linear.x = 0.0;
    V_.linear.y = 0.0;
    V_.linear.z = 0.0;
    V_.angular.x = 0.0;
    V_.angular.y = 0.0;
    V_.angular.z = 0.0;

    // Sets the covariance matrix to zeros and sets the localization type to
    // velocity
    pre_init();
  }

  //////////////////////////////////////////////////////////////////////////////
  //
  // VelocityLocalizationElement()
  //
  //////////////////////////////////////////////////////////////////////////////
  Velocity3DLocalizationElement::Velocity3DLocalizationElement(
      const Velocity3DLocalizationElement& elem) : LocalizationElement(elem)
  {
    V_ = elem.V_;

    Cov_.copy(elem.Cov_);

    orig_stamp_ = elem.orig_stamp_;

    if (!Velocity3DLocalizationElement::update_matrix_initialized_)
    {
      Velocity3DLocalizationElement::update_matrix_initialized_ = true;
      four_by_six_update_matrix_ = LaGenMatDouble::zeros(4, 6);
      four_by_six_update_matrix_(0, 0) = 1.0;  // Associated with v
      four_by_six_update_matrix_(1, 5) = 1.0;  // Associated with wz --> wz is correct in this place
      four_by_six_update_matrix_(2, 4) = 1.0;  // Associated with wy
      four_by_six_update_matrix_(3, 3) = 1.0;  // Associated with wx --> wx is correct in this place
    }
  }




  //////////////////////////////////////////////////////////////////////////////
  //
  // VelocityLocalizationElement()
  //
  //////////////////////////////////////////////////////////////////////////////
  Velocity3DLocalizationElement::Velocity3DLocalizationElement(
      const geometry_msgs::TwistWithCovarianceStamped& Vinit)
  {
    pre_init();
    set_velocity_data(Vinit);
  }


  void Velocity3DLocalizationElement::pre_init()
  {
    Cov_ = LaGenMatDouble::zeros(6, 6);
    if (!Velocity3DLocalizationElement::update_matrix_initialized_)
    {
      Velocity3DLocalizationElement::update_matrix_initialized_ = true;
      four_by_six_update_matrix_ = LaGenMatDouble::zeros(4, 6);
      four_by_six_update_matrix_(0, 0) = 1.0;  // Associated with v
      four_by_six_update_matrix_(1, 5) = 1.0;  // Associated with wz --> wz is correct in this place
      four_by_six_update_matrix_(2, 4) = 1.0;  // Associated with wy
      four_by_six_update_matrix_(3, 3) = 1.0;  // Associated with wx --> wx is correct in this place
    }

    setLocType(sumet_state_estimator::Loc_Velocity);
  }

  //////////////////////////////////////////////////////////////////////////////
  //
  // convert_cov()
  //
  //////////////////////////////////////////////////////////////////////////////
  void Velocity3DLocalizationElement::convert_cov(const boost::array<double,
                                                  36>& cov_in)
  {
    const int32_t row_len = 6;
    for (int32_t i = 0; i < row_len; i++)
    {
      for (int32_t j = 0; j < row_len; j++)
      {
        // cov_in is a 36 element row major boost array (6x6 array)
        Cov_(i, j) = cov_in[i*row_len + j];
      }
    }
  }

  //////////////////////////////////////////////////////////////////////////////
  //
  // get_twist()
  //
  //////////////////////////////////////////////////////////////////////////////
  void Velocity3DLocalizationElement::get_twist(
      geometry_msgs::TwistWithCovarianceStamped& twist) const
  {
    // TODO(kkozak): probably need to add a valid frame_id
    twist.header.stamp = this->getTimestamp();
    twist.twist.twist = V_;
    boost::array<double, 36>& cov = twist.twist.covariance;
    for (int i = 0; i < 6; i++)
    {
      for (int j = 0; j < 6; j++)
      {
        cov[i*6 + j] = Cov_(i, j);
      }
    }
  }

  //////////////////////////////////////////////////////////////////////////////
  //
  // run_update_step()
  //
  //////////////////////////////////////////////////////////////////////////////
  void Velocity3DLocalizationElement::run_update_step(
    const tf::Transform& T_cur,
    const LaGenMatDouble& Cov_cur,
    tf::Transform& T_out,
    LaGenMatDouble& Cov_out,
    double DT,
    bool verbose)
  {
    tf::Vector3 X_0 = T_cur.getOrigin();
    tf::Vector3 X_out(0.0,
                    0.0,
                    0.0);

    tf::Quaternion Q_0 = T_cur.getRotation();
    double roll;
    double pitch;
    double theta;

    tf::Transform(Q_0).getBasis().getRPY(roll, pitch, theta);

    theta = sumet_util::MathUtil::FixAngle0to2Pi(theta);

    tf::Vector3 rpy(roll,
                  pitch,
                  theta);

    tf::Quaternion Q_out = tf::Quaternion::getIdentity();

    tf::Vector3 v_in(V_.linear.x,
                   V_.linear.y,
                   V_.linear.z);

    tf::Vector3 w_in(V_.angular.x,
                   V_.angular.y,
                   V_.angular.z);

    // Predict linear and angular motion for DT time step
    sumet_state_estimator::Predict6DofForDT(DT,
                                                Q_0,
                                                X_0,
                                                v_in,
                                                w_in,
                                                X_out,
                                                Q_out);

    T_out = tf::Transform::getIdentity();
    T_out.setOrigin(X_out);
    T_out.setRotation(Q_out);


    LaGenMatDouble Vmat;
    LaGenMatDouble G;
    sumet_state_estimator::gen_G_and_V_6DOF(X_0,
                                                rpy,
                                                v_in,
                                                w_in,
                                                DT,
                                                G,
                                                Vmat);

    // We'll avoid accidentally altering the covariance by copying to a
    // temporary matrix
    LaGenMatDouble cov_temp = Cov_;

    // First we'll contract the 6x6 covariance matrix to a 4x4 matrix that keeps
    // only the elements of the matrix that should be valid (the ones associated
    // with v, wx, wy, wz)
    LaGenMatDouble Cov4x4_temp(four_by_six_update_matrix_.rows(),
                                                       cov_temp.cols());
    Blas_Mat_Mat_Mult(four_by_six_update_matrix_,
                      cov_temp,
                      Cov4x4_temp);

    LaGenMatDouble Cov4x4(Cov4x4_temp.rows(),
                          four_by_six_update_matrix_.rows());

    Blas_Mat_Mat_Trans_Mult(Cov4x4_temp,
                            four_by_six_update_matrix_,
                            Cov4x4);

    double age = this->get_age();
    if (age > 0.1)
    {
      // in the worst case, we could slam on the brakes from full speed and
      // decelerate at an approximately constant rate of about 0.7g (~7 m/s^2),
      // which is an over-conservative estimate.  This would mean that our
      // velocity would be off by 7 m/s * t, the square of which is what we'll
      // scale the covariance by:
      double temp_scale = std::max(7.0 * age, 1.0);

      cov_temp.scale(temp_scale * temp_scale);

      ROS_ERROR_THROTTLE(
        1.0,
        "Velocity data stream was lost... age = %f. Now scaling the "
        "covariance by %f",
        age,
        temp_scale);

      ROS_ERROR_THROTTLE(
        1.0,
        "Timestamp = %f, Original Timestamp = %f, Ros Time = %f",
        this->getTimestamp().toSec(),
        this->orig_stamp_.toSec(),
        ros::Time::now().toSec());
    }


    // Cov_out = G*Sigma1*G' + V*M1*V';
    LaGenMatDouble CovTemp1(Vmat.rows(), Cov4x4.cols());

    LaGenMatDouble CovTemp2(Vmat.rows(), Vmat.rows());

    Blas_Mat_Mat_Mult(Vmat,
                      Cov4x4,
                      CovTemp1);

    Blas_Mat_Mat_Trans_Mult(CovTemp1,
                            Vmat,
                            CovTemp2);

    LaGenMatDouble CovTemp3(G.rows(), G.rows());

    LaGenMatDouble CovTemp4(G.rows(), G.rows());

    Blas_Mat_Mat_Mult(G,
                      Cov_cur,
                      CovTemp3);

    Blas_Mat_Mat_Trans_Mult(CovTemp3,
                            G,
                            CovTemp4);
    // Pitch error affects the variance of altitude but not vice versa, so we'll
    // add a separate term here to account for the unidirectional relationship;
    // the pitch influenced elevation error propagation has been removed from
    // the prediction step.
    // TODO(kkozak): The value should be auto-calibrated in the future.

    // Previously, the code here created a temporary zero matrix, set a single value
    // in it, then add that to the other temporary matrices; is there any reason to do
    // that rather than just add a single value and skip the matrix allocation?
    //LaGenMatDouble CovTemp5 = LaGenMatDouble::zeros(G.rows(),
    //                                                G.rows());

    double pitch_induced_elevation_variance_per_meter = 0.01;
    //CovTemp5(2, 2) = v_in.x()*DT*pitch_induced_elevation_variance_per_meter;

    Cov_out = CovTemp4 + CovTemp2;// + CovTemp5;
    Cov_out(2,2) += v_in.x()*DT*pitch_induced_elevation_variance_per_meter;
  }


  void Velocity3DLocalizationElement::run_update_step(
    const LaVectorDouble& X_cur,
    const LaGenMatDouble& Cov_cur,
    LaVectorDouble& X_out,
    LaGenMatDouble& Cov_out,
    double DT,
    bool verbose)
  {
    X_out = LaVectorDouble::zeros(6, 1);
    Cov_out = LaVectorDouble::eye(6, 6);
    // After writing the other "run_update_step" I realized that we needed to
    // maintain the same interface, so that's being done with this somewhat
    // inefficient shuffling of variables
    tf::Vector3 X(X_cur(0),
                X_cur(1),
                X_cur(2));

    tf::Quaternion Q;

    Q.setRPY(X_cur(5),
             X_cur(4),
             X_cur(3));

    tf::Transform T(Q, X);

    // Create T_out for storage of transform results
    tf::Transform T_out;

    // Call other run_update_step function  Cov_out gets filled directly here
    run_update_step(T,
                    Cov_cur,
                    T_out,
                    Cov_out,
                    DT,
                    verbose);

    // Fill in the output values
    X = T_out.getOrigin();
    X_out(0) = X.x();
    X_out(1) = X.y();
    X_out(2) = X.z();
    T_out.getBasis().getRPY(X_out(5),
                            X_out(4),
                            X_out(3));
  }

  //////////////////////////////////////////////////////////////////////////////
  //
  // report_data()
  //
  //////////////////////////////////////////////////////////////////////////////
  void Velocity3DLocalizationElement::report_data() const
  {
    ROS_INFO("Time: %f, Type: Velocity, v = (%f, %f, %f), w = (%f, %f, %f)",
             this->getTimestamp().toSec(),
             V_.linear.x, V_.linear.y, V_.linear.z,
             V_.angular.x, V_.angular.y, V_.angular.z);
  }

  //////////////////////////////////////////////////////////////////////////////
  //
  // set_covariance()
  //
  //////////////////////////////////////////////////////////////////////////////
  void Velocity3DLocalizationElement::set_covariance(const LaGenMatDouble & cov)
  {
    Cov_ = cov;
  }

  //////////////////////////////////////////////////////////////////////////////
  //
  // set_velocities()
  //
  //////////////////////////////////////////////////////////////////////////////
  void Velocity3DLocalizationElement::set_velocities(
      const geometry_msgs::Twist& Vin)
  {
    V_ = Vin;
  }


  void Velocity3DLocalizationElement::set_velocity_data(
      const geometry_msgs::TwistWithCovarianceStamped& Vin)
  {
    set_velocities(Vin.twist.twist);
    convert_cov(Vin.twist.covariance);
    setTimestamp(Vin.header.stamp);

    // This step is essential! The original timestamp must be set before the
    // localization element gets inserted into the LocalizationQueue.  Doing it
    // during this call helps to ensure that it only happens when we receive the
    // original Velocity measurement message.
    // TODO(kkozak): May need to figure out a way to write a unit test to make
    // sure that this functionality doesn't get broken later down the line.
    this->set_orig_timestamp(Vin.header.stamp);
  }


  //////////////////////////////////////////////////////////////////////////////
  //
  // get_age()
  //
  //////////////////////////////////////////////////////////////////////////////
  double Velocity3DLocalizationElement::get_age() const
  {
    return ((this->getTimestamp() - this->orig_stamp_).toSec());
  }


  //////////////////////////////////////////////////////////////////////////////
  //
  // set_orig_timestamp()
  //
  //////////////////////////////////////////////////////////////////////////////
  void Velocity3DLocalizationElement::set_orig_timestamp(
      const ros::Time& orig_stamp)
  {
    this->orig_stamp_ = orig_stamp;
  }
}
