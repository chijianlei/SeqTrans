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
// Contract No.  N00178-11-C-1005
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

/**
 * \file
 *
 * This node reformats absolute pose
 * data (which may be incomplete) into pose messages for the state estimator
 * and publishes it immediately.  Aggregating the data before using it in the
 * state estimator allows for the absolute position update interface in the
 * state estimator to be abstracted away from needing specific messages into
 * a single generic type of message (PoseWithCovarianceStamped).  The design of
 * the EKF is such that there is no need to publish fully populated pose
 * messages, setting high covariances (above 1e12 on the diagonal – note that
 * independent variances are assumed) is all that is needed to indicate to the
 * state estimator that a particular field of the message is not valid.
 *
 *   - <b>Subscribed Topics</b>
 *      - \e gps <tt>[marti_gps_common::GPSFix]</tt> -
 *      This is the expected message from the Novatel GPS node.
 *      - \e imu <tt>[sensor_msgs::Imu]</tt> -
 *      This is the expected message from the Microstrain IMU node, but it can
 *      be provided by any node that publishes IMU data.  Note that pitch and
 *      roll (extracted from the quaternion) are the only elements used from
 *      the imu.
 *
 *   - <b>Published Topics</b>
 *      - \e AbsolutePoseMeasurement <tt>[PoseWithCovarianceStamped]</tt> -
 *      The aggregated pose with covariance measurement.  The covariance, which
 *      currently is forced to be diagonal, will indicate which components of
 *      the measurement are valid.
 */

// Boost Libraries
#include <boost/algorithm/string.hpp>
#include <boost/shared_ptr.hpp>

// ROS Libraries
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <tf/tf.h>
#include <gps_common/GPSFix.h>

// SUMET Libraries
#include <sumet_util/math_util.h>

// MARTI Libraries
#include <swri_transform_util/local_xy_util.h>
#include <swri_transform_util/transform_manager.h>

// SwRI Libraries
#include <swri_roscpp/subscriber.h>
#include <swri_roscpp/parameters.h>
#include <marti_dbw_msgs/TransmissionFeedback.h>

namespace sumet_state_estimator
{
/**
 * @brief Main class for performing capture aggregation of position measurements
 *
 * This class works with the sumet_position_meas_agg_node to capture relevant
 * (expected) absolute position measurements then republishes them as
 * PoseWithCovariance messages.  This aggregation is performed in order to
 * condense the measurement input stream into the state estimator into a single
 * message (although the underlying data may come from different sources.
 */
    class PoseAggregator : public nodelet::Nodelet
    {
    private:
        // Subscriptions
        swri::Subscriber Novatel_Subscriber_;
        swri::Subscriber Imu_Subscriber_;
        swri::Subscriber transmission_sense_sub_;

        // Publishers
        ros::Publisher pose_publisher_;

        // Used to delay initialization
        ros::WallTimer init_timer_;

        // Transform utils
        swri_transform_util::TransformManager tf_manager_;
        swri_transform_util::LocalXyWgs84UtilPtr local_xy_;

        // Diagnostics
        boost::shared_ptr<diagnostic_updater::Updater> diagnostic_updater_;
        ros::Timer diagnostic_timer_;
        ros::Time last_gps_update_;
        ros::Time last_imu_update_;

        // Diagnostics parameters
        double gps_timeout_;
        double imu_timeout_;

        // Parameters
        double nominal_pitch_;
        double nominal_roll_;

        bool in_reverse_;

      
        void transmission_sense_cb(
          const marti_dbw_msgs::TransmissionFeedbackConstPtr& msg)
        {
          in_reverse_ = msg->reverse;
        }

        /**
         * @brief      Callback for novatel gps messages
         *
         * This function repackages and republishes GPSFix messages from the Novatel
         * driver as VehiclePose messages.
         *
         * @param[in]    GPS            GPSFix message from the novatel (the driver
         *                              node that interfaces with the Novatel GPS
         *                              receiver)
         */
        void Novatel_cb(const gps_common::GPSFixConstPtr& GPS)
        {
            if (local_xy_->Initialized())
            {
                // We will trigger the absolute position aggregate measurement output off of
                // this message

                // Convert the GPSFix message to a Pose message

                geometry_msgs::PoseWithCovarianceStampedPtr cur_pose(new geometry_msgs::PoseWithCovarianceStamped());
                GPSFix2VehiclePose(*cur_pose, *GPS);

                if (in_reverse_)
                {
                    tf::Quaternion quat;
                    tf::quaternionMsgToTF(cur_pose->pose.pose.orientation, quat);
                    tf::Transform transform1(quat);

                    // Create a 180.0 rotation.
                    tf::Vector3 axis(0.0, 0.0, 1.0);
                    tf::Transform transform2(tf::Quaternion(axis, M_PI));
                    transform1 = transform2 * transform1;

                    tf::quaternionTFToMsg(transform1.getRotation(),
                                          cur_pose->pose.pose.orientation);
                }

                pose_publisher_.publish(cur_pose);

                // Diagnostics
                last_gps_update_ = GPS->header.stamp;
            }
            else
            {
                NODELET_ERROR("Position aggregator waiting for local origin");
            }
        }

        /**
         * @brief      This function encapsulates all of the manipulations to come up
         *             with reasonable variances for the various state variables
         *
         * @param[in]  GPS            A pointer to a GPSFix message from the Novatel
         * @param[out] yaw_var       The heading variance (in rad^2)
         * @param[out] horz_var          The horizontal variance (assume x_var = y_var)
         * @param[out] vert_var          The vertical variance
         */
        void get_gps_variances(const gps_common::GPSFix& GPS, double& yaw_var,
                               double& horz_var, double& vert_var) const
        {
            // GPS provides measurements (or direct estimates) of 4 states: x, y (from
            // lat and lon), z (from altitude), and yaw (heading).  For the single
            // antenna Novatel receiver setup in EV-1, speed is the main variable
            // affecting accuracy of heading.  Horizontal positioning is pretty much the
            // same for all speeds, but raising the variance at low speeds helps to
            // reduce the jumpiness of the estimates Set the planar positioning variance
            // based on speed (we don't want the really low speed jumping around of
            // position measurements to affect our state estimate
            double speed = GPS.speed;

            horz_var = 0.2;
            vert_var = 100.0;

            if (speed < 0.5)
            {
                horz_var = 1.0e2;
                vert_var = 1.0e6;
            }
            else
            {
                horz_var = 1.0;  // Raised from 0.02 to be less aggressive in
                // utilizing position updates
                vert_var = 40.0;  // Raised from 1.0 to be less aggressive in
                // utilizing position updates
            }

            if (GPS.hdop < 3.0)
            {
                horz_var *= GPS.hdop; // * 10.0;
                // yes, multiply by hdop rather than vdop
                vert_var *= GPS.hdop; // * 100.0;
            }
            else
            {
                horz_var *= GPS.hdop * 1000.0; // * 10.0;
                // yes, multiply by hdop rather than vdop
                vert_var *= GPS.hdop * 1000.0; // * 100.0;
            }

            // For heading variance, we'll set different levels depending on the speed.
            // With a one antenna receiver, when the vehicle is stationary or moving
            // very slowly, the heading will be essentially random (and uncorrelated
            // with the actual heading).  In this case we want to set variance very
            // high.  We will tend to trust the heading measurements more as speed
            // increases -- this is because the heading measurement is essentially just
            // an estimate of heading based on the vector difference between two
            // subsequent position measurements.

            yaw_var = 0.01;  // init

            const double low_speed_thresh = 0.5;
            const double switch_speed1 = 2.0;
            const double switch_speed2 = 10.0;
            const double max_var = 1000.0;
            const double knee1_var = 20.0;  // Raised from 10.0 to be less
            // aggressive in utilizing heading
            // updates
            const double knee2_var = 1.0;  // Raised from 0.05 to be less
            // agressive in utilizing heading
            // updates

            if (speed < low_speed_thresh)
            {
                yaw_var = sumet_util::_large_variance;
            }
            else if (speed < switch_speed1)
            {
                // for very low speeds we'll linearly interpolate between max_var and
                // knee1_var
                yaw_var = max_var - ((max_var - knee1_var) / switch_speed1) * speed;
            }
            else if (speed < switch_speed2)
            {
                // for modest speeds we'll linearly interpolate between knee1_var and
                // knee2_var
                yaw_var = knee1_var - ((knee1_var - knee2_var) / (switch_speed2
                                                                  - switch_speed1)) * (speed - switch_speed1);
            }
            else
            {
                // for speeds above switch_speed2, we'll just use a constant, minimum
                // variance
                yaw_var = knee2_var;
            }
        }

        /**
         * @brief      Converts GPSFix messages to VehiclePose messages
         *
         * @param[out]    msg_out            Pose message output
         * @param[in]     gps_fix            GPSFix input
         */
        void GPSFix2VehiclePose(geometry_msgs::PoseWithCovarianceStamped& msg_out,  // NOLINT
                                const gps_common::GPSFix& gps_fix)
        {

            msg_out.header = gps_fix.header;

            // We'll tag the frame id with gps so that we can filter out these updates
            // if necessary
            msg_out.header.frame_id = "swri_gps";

            geometry_msgs::Point& X = msg_out.pose.pose.position;

            local_xy_->ToLocalXy(gps_fix.latitude, gps_fix.longitude, X.x, X.y);

            X.z = gps_fix.altitude;

            // Convert heading angle from NED to ENU representation
            double tempAngle = 90.0 - gps_fix.track;

            // Convert heading to radians and fix to 0-2PI
            double yaw = sumet_util::MathUtil::FixAngle0to2Pi(
                sumet_util::MathUtil::ToRadians(tempAngle));

            double pitch = 0.0;
            double roll = 0.0;
            double pitch_var = sumet_util::_large_variance;
            double roll_var = sumet_util::_large_variance;

            tf::Quaternion Q_out = tf::Quaternion::getIdentity();
            Q_out.setRPY(roll, pitch, yaw);

            tf::Vector3 X_out(X.x, X.y, X.z);

            setPose(msg_out.pose.pose, X_out, Q_out);

            double yaw_var = sumet_util::_large_variance;
            double horz_var = sumet_util::_large_variance;
            double vert_var = sumet_util::_large_variance;

            get_gps_variances(gps_fix, yaw_var, horz_var, vert_var);

            setDiagCov(msg_out.pose.covariance,
                       horz_var, horz_var, vert_var,  // x_var,   y_var,     z_var
                       yaw_var, pitch_var, roll_var);  // yaw_var, pitch_var, roll_var
        }

        void Imu_msg_handler(const sensor_msgs::Imu& msg)
        {
            last_imu_update_ = ros::Time::now();
            geometry_msgs::PoseWithCovarianceStampedPtr cur_pose(new geometry_msgs::PoseWithCovarianceStamped());

            double yaw = 0.0;
            double pitch = 0.0;
            double roll = 0.0;
            double yaw_var = sumet_util::_large_variance;
            double pitch_var = sumet_util::_large_variance;
            double roll_var = sumet_util::_large_variance;

            tf::Quaternion Q(msg.orientation.x,
                             msg.orientation.y,
                             msg.orientation.z,
                             msg.orientation.w);

            // Rotate by 180 around x-axis to get the axes into the upright orientation
            Q = Q * tf::Quaternion(tf::Vector3(1.0, 0.0, 0.0), sumet_util::_pi);

            tf::Transform(Q).getBasis().getRPY(roll, pitch, yaw);


            getCov3x3DiagVals(msg.orientation_covariance, roll_var,
                              pitch_var, yaw_var);

            // We don't know yaw with any certainty from this message, regardless of the
            // variance attributed to it in the message, so we'll max out the variance
            // for that element
            yaw_var = sumet_util::_large_variance;

            tf::Quaternion Q_out = tf::Quaternion::getIdentity();
            Q_out.setRPY(roll - nominal_roll_, pitch - nominal_pitch_, yaw);

            // IMU does not provide position information, so the vector is arbitrary:
            tf::Vector3 X_out(0, 0, 0);

            setPose(cur_pose->pose.pose, X_out, Q_out);

            // The IMU does not give us any positional data, so we'll max out the
            // variances associated with horizontal and vertical position components
            double horz_var = sumet_util::_large_variance;
            double vert_var = sumet_util::_large_variance;

            // After this operation cov should be a diagonal matrix with the following
            // values on the diagonal: {MAX, MAX, MAX, MAX, pitch_var, roll_var)
            setDiagCov(cur_pose->pose.covariance,
                       horz_var, horz_var, vert_var,  // x_var, y_var, z_var
                       yaw_var, pitch_var, roll_var);  // yaw_var, pitch_var, roll_var

            // Set the timestamp to the current message time:
            cur_pose->header.stamp = msg.header.stamp;

            // We'll tag the frame id with imu so that we can filter out these updates
            // if necessary
            cur_pose->header.frame_id = "swri_imu";

            // The message should now be valid
            pose_publisher_.publish(cur_pose);
        }

        void Imu_cb(const sensor_msgs::ImuConstPtr& msg)
        {
            Imu_msg_handler(*msg);
        }

        /**
         * @brief        Function that handles all of the subscriptions
         */
        void subscribe_to_topics()
        {
            ros::NodeHandle& nh = getNodeHandle();
            // Subscribes to the novatel GPS message
            Novatel_Subscriber_ = swri::Subscriber(nh, "gps", 100,
                                                &PoseAggregator::Novatel_cb, this);

            // Subscribes to and IMU message (for pitch and roll information)
            Imu_Subscriber_ = swri::Subscriber(nh, "imu", 100,
                                            &PoseAggregator::Imu_cb, this);

            transmission_sense_sub_ = swri::Subscriber(nh,
                "/vehicle_interface/transmission_sense", 1,
                &PoseAggregator::transmission_sense_cb, this);
        }

        /**
         * @brief      Function that handles all of the topic advertisements
         */
        void advertise_topics()
        {
            ros::NodeHandle& nh = getNodeHandle();
            pose_publisher_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(
                "AbsolutePoseMeasurement", 100);
        }

        /**
         * @brief      Initialization function
         */
        void init_node()
        {
            ros::NodeHandle& pnh = getPrivateNodeHandle();

            // Initialize the diagnostics
            init_diagnostics();

            swri::param(pnh,"nominal_pitch", nominal_pitch_, 0.0);
            swri::param(pnh,"nominal_roll", nominal_roll_, 0.0);

            nominal_pitch_ = sumet_util::MathUtil::ToRadians(nominal_pitch_);
            nominal_roll_ = sumet_util::MathUtil::ToRadians(nominal_roll_);


            last_gps_update_ = ros::TIME_MIN;
            last_imu_update_ = ros::TIME_MIN;

            swri::param(pnh,"gps_timeout", gps_timeout_, 5.0);
            swri::param(pnh,"imu_timeout", imu_timeout_, 0.5);

            // Initialize the transform manager to handle GPS conversions
            local_xy_ = boost::make_shared<swri_transform_util::LocalXyWgs84Util>();
        }


        /////////////////////////// //
        //                          //
        //       Diagnostics        //
        //                          //
        /////////////////////////// //


        void GpsDiagnostic(diagnostic_updater::DiagnosticStatusWrapper& status)  // NOLINT
        {
            ros::Time cur_time = ros::Time::now();
            double time_since_last_gps_update = (cur_time - last_gps_update_).toSec();

            if (time_since_last_gps_update < gps_timeout_)
            {
                status.summary(0, "GPS is active and available in pose aggregator");
            }
            else
            {
                char buff[1024];
                snprintf(buff, sizeof(buff), "Last GPS update %f seconds ago",
                         time_since_last_gps_update);
                status.summary(2, std::string(buff));
            }
            status.add("Time since last GPS Update", time_since_last_gps_update);
        }

        void ImuDiagnostic(diagnostic_updater::DiagnosticStatusWrapper& status)  // NOLINT
        {
            ros::Time cur_time = ros::Time::now();
            double time_since_last_imu_update = (cur_time - last_imu_update_).toSec();

            if (time_since_last_imu_update < imu_timeout_)
            {
                status.summary(status.OK, "IMU is active and available in pose aggregator");
            }
            else
            {
                status.summary(status.ERROR, "IMU has timed out in pose aggregator");
            }
            status.add("Time since last IMU update", time_since_last_imu_update);
        }

        void communicationDiagnostic(diagnostic_updater::DiagnosticStatusWrapper& status)
        {
            status.summary(diagnostic_updater::DiagnosticStatusWrapper::OK, "No errors reported.");
            Novatel_Subscriber_.appendDiagnostics(status, "Novatel",
                                                  swri::Subscriber::DIAG_MOST);
            Imu_Subscriber_.appendDiagnostics(status, "IMU",
                                              swri::Subscriber::DIAG_MOST);
            transmission_sense_sub_.appendDiagnostics(status, "Transmission",
                                                      swri::Subscriber::DIAG_MOST);
        }

        void RunDiagnostics(const ros::TimerEvent& te)
        {
            diagnostic_updater_->update();
        }

        void init_diagnostics()
        {
            diagnostic_updater_->setHardwareID("pose_aggregator");

            diagnostic_updater_->add("GPS Status",
                                    this,
                                    &PoseAggregator::GpsDiagnostic);

            diagnostic_updater_->add("IMU Status",
                                    this,
                                    &PoseAggregator::ImuDiagnostic);

            diagnostic_updater_->add("Communications",
                                    this,
                                    &PoseAggregator::communicationDiagnostic);

            ros::NodeHandle& nh = getPrivateNodeHandle();
            diagnostic_timer_ =
                nh.createTimer(ros::Duration(1),
                                &PoseAggregator::RunDiagnostics,
                                this);
        }

        /////////////////////////// //
        //                          //
        //  HELPER FUNCTIONS BELOW  //
        //                          //
        /////////////////////////// //

        /**
         * @brief      Sets a pose message from a tf::Vector3 and a tf::Quaternion
         *
         * @param[out] p              The output pose message
         * @param[in]  X              The position
         * @param[in]  Q              The orientation
         */
        void setPose(geometry_msgs::Pose& p, const tf::Vector3& X,
                     const tf::Quaternion& Q)
        {
            p.position.x = X.x();
            p.position.y = X.y();
            p.position.z = X.z();

            p.orientation.x = Q.x();
            p.orientation.y = Q.y();
            p.orientation.z = Q.z();
            p.orientation.w = Q.w();
        }

        /**
         * @brief        Sets the diagonal values of a 3x3 covariance matrix
         *
         * @param[out]   cov_out        Output 3x3 matrix in row-major boost array
         *                              format
         * @param[in]    v1             First diagonal value (variance)
         * @param[in]    v2             First diagonal value (variance)
         * @param[in]    v3             First diagonal value (variance)
         * @param[in]    v4             First diagonal value (variance)
         * @param[in]    v5             First diagonal value (variance)
         * @param[in]    v6             First diagonal value (variance)
         */
        void setDiagCov(boost::array<double, 36>& cov_out, double v1 =  // NOLINT
        sumet_util::_large_variance, double v2 = sumet_util::_large_variance,
                        double v3 = sumet_util::_large_variance, double v4 =
        sumet_util::_large_variance, double v5 = sumet_util::_large_variance,
                        double v6 = sumet_util::_large_variance)
        {
            cov_out.assign(0.0);

            cov_out[0] = v1;
            cov_out[7] = v2;
            cov_out[14] = v3;
            cov_out[21] = v4;
            cov_out[28] = v5;
            cov_out[35] = v6;
        }

        /**
         * @brief      Gets the diagonal values of the 3x3 covariance matrix
         *
         * @param[in]  cov_in         The covariance matrix (boost array)
         * @param[out] v1             cov_in(0,0)
         * @param[out] v2             cov_in(1,1)
         * @param[out] v3             cov_in(2,2)
         */
        void getCov3x3DiagVals(const boost::array<double, 9>& cov_in, double& v1,
                               double& v2, double& v3) const
        {
            v1 = cov_in[0];
            v2 = cov_in[4];
            v3 = cov_in[8];
        }
        
        void onInit()
        {
            double initialization_delay = 1.0;
            swri::param(getPrivateNodeHandle(),"initialization_delay_s", initialization_delay, 1.0);
            init_timer_ = getNodeHandle().createWallTimer(ros::WallDuration(initialization_delay),
                                                          &PoseAggregator::initialize,
                                                          this,
                                                          true);
        }

        void initialize(const ros::WallTimerEvent&)
        {
            diagnostic_updater_ = boost::make_shared<diagnostic_updater::Updater>(
                getNodeHandle(), getPrivateNodeHandle(), getName());
            // Initialize node
            init_node();

            // Publish topics
            advertise_topics();

            // Subscribe to topics
            subscribe_to_topics();
        }


    public:
        /**
         * @brief      Constructor
         */
        explicit PoseAggregator() :
            in_reverse_(false)
        {
        }
    };
}  // namespace sumet_state_estimator

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sumet_position_meas_agg_node",
              ros::init_options::AnonymousName);

    ros::NodeHandle n;

    sumet_state_estimator::PoseAggregator PosMeas();

    ros::spin();

    return 0;
}

// Register nodelet plugin
#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(sumet_state_estimator,
                        sumet_3D_position_aggregator_nodelet,
                        sumet_state_estimator::PoseAggregator,
                        nodelet::Nodelet)
