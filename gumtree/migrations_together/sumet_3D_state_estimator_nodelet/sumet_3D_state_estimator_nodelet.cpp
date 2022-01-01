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

/**
 * \file
 *
 * This node carries out the core localization task, where
 * motion is predicted based on velocity measurements, and where the sensor
 * data is ultimately fused into an estimate of location.  This node performs
 * both a fully relative localization state estimate (with only pitch and roll
 * as absolute corrections), and a fully integrated localization state estimate
 * (with GPS and pitch and roll absolute updates) in parallel.  The fully
 * relative localization state estimate is used as the basis for all perception
 * and control, i.e. obstacles locations, path location and geometry, etc. are
 * specified in this frame – the near-field frame.  The fully integrated state
 * estimate, on the other hand, is used to track the “best estimate” of the
 * vehicle’s absolute position.  With these estimates of pose, data from the
 * world frame (such as a far-field route) can be projected into the
 * near-field frame and used for navigation and control.  The fully integrated
 * localization state estimator also handles loss of GPS smoothly and maintains
 * a best estimate of the vehicle’s absolute location during GPS dropouts.
 *
 *   - <b>Subscribed Topics</b>
 *      - \e VehicleVelocity6D <tt>[geometry_msgs::TwistWithCovarianceStamped]</tt> -
 *      This is the message expected from the sumet_velocity3D_aggregator_node.
 *      - \e AbsolutePoseMeasurement <tt>[geometry_msgs::PoseWithCovarianceStamped]</tt> -
 *      This is the message expected from the sumet_3D_position_aggregator_node.
 *
 *   - <b>Published Topics</b>
 *      - \e near_field_odom <tt>[nav_msgs::Odometry]</tt> -
 *      An odometry message with the fully relative odometry in the near field
 *      frame.
 *      - \e far_field_odom <tt>[nav_msgs::Odometry}</tt> -
 *      An odometry message with the absolute odometry (i.e. from the full
 *      state estimate) in the far field frame
 *      - \e utm_odom <tt>[nav_msgs::Odometry]</tt> - An
 *      odometry message with the odometry in the UTM frame.  This message is
 *      primarily intended for display of vehicle path/position in mapviz, but
 *      it does provide valid UTM coordinates.
 *      - \e gps_utm_odom <tt>[nav_msgs::Odometry]</tt> - An odometry message
 *      taken from the raw GPS signal in the UTM frame, intended primarily
 *      for visualization.
 *      - \e utm_zone <tt>[std_msgs::UInt8]</tt> - The current UTM zone,
 *      published with a latched publisher.
 *      - \e utm_band <tt>[std_msgs::Char]</tt> - The current UTM band,
 *      published with a latched publisher.
 *
 *   - <b>Published Transforms</b>
 *      \dot
 *        digraph TFs {
 *        utm [ label="utm"];
 *        far_field [ label="far_field"];
 *        near_field [ label="near_field"];
 *        veh_near_field [ label="veh_near_field"];
 *        utm -> far_field [ arrowhead="open", style="solid"];
 *        far_field -> near_field [ arrowhead="open", style="solid" label="Abs_To_Rel_tf_"];
 *        near_field -> veh_near_field [ arrowhead="open", style="solid"  label="Rel_LocalXY_tf_"];
 *        }
 *      \enddot
 *      - \e /far_field_id <tt>[parent: /utm]</tt> - An Absolute LocalXY frame
 *      (rectified with an origin at the SwRI test track). (ID set by param)
 *      - \e /near_field_id <tt>[parent: /far_field_id]</tt> - The frame in
 *      which the purely relative state estimation is performed.
 *      - \e /veh_near_field <tt>[parent: /near_field]</tt> - The frame
 *      attached to the vehicle in the near_field -- The transform between the
 *      veh_near_field and the near_field exactly represents the purely
 *      relative state estimate (which always starts from the origin of the
 *      near_field frame). (ID set by parameter)
 *
 *   - <b>Required Transforms</b>
 *     - \e /gps_frame_id - The frame of the gps antenna. (ID set by param.)
 *
 *   - \b Parameters
 *      - \e ~/debug_on <tt>[bool]</tt> - If true, a small amount
 *      of additional debugging information is printed to the console. [false]
 *      - \e ~/publish_diagnostics <tt>[bool]</tt> - If true, diagnostics are
 *      published for the robot monitor. [true]
 *      - \e ~/gps_level1_timeout <tt>[double]</tt> - [0.2]
 *      - \e ~/gps_level2_timeout <tt>[double]</tt> - [5.0]
 *      - \e ~/verbose <tt>[bool]</tt> - If true, output messages are
 *      more verbose. [false]
 *      - \e ~/near_field_id <tt>[string]</tt> - Name of the near
 *      field coordinate frame. ["/near_field"]
 *      - \e ~/near_field_veh_id <tt>[string]</tt> - Name of frame
 *      attached to the vehicle in the near field. ["/veh_near_field"]
 *      - \e ~/far_field_id <tt>[string]</tt> - Name of the far field
 *      frame. ["/far_field"]
 *      - \e ~/gps_frame_id <tt>[string]</tt> - Name of the frame
 *      that the gps pose points to. ["/gps"]
 */

// Standard C++ Libraries
#include <string>
#include <algorithm>

// Boost Libraries
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/circular_buffer.hpp>

// ROS Libraries
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <std_msgs/Char.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <diagnostic_updater/diagnostic_updater.h>

// SUMET Libraries
#include <sumet_state_estimator/LocalizationQueue6DOF.h>
#include <sumet_util/math_util.h>
#include <sumet_util/time_util.h>
#include <sumet_state_estimator/PoseTransformer.h>

// MARTI Libraries
#include <swri_transform_util/local_xy_util.h>
#include <swri_transform_util/utm_util.h>
#include <swri_transform_util/transform_manager.h>

// SwRI Libraries
#include <swri_roscpp/parameters.h>
#include <swri_roscpp/publisher.h>
#include <swri_roscpp/subscriber.h>

#define DEFAULT_SAMPLING_FREQUENCY                                      50.0

namespace sumet_state_estimator
{
    struct OdomPair
    {
        nav_msgs::OdometryPtr ff_odom;
        nav_msgs::OdometryPtr gps_odom;
    };

/**
 * SUMET state estimator class. See sumet_3D_state_estimator_node.cpp
 */
    class StateEst3DClass : public nodelet::Nodelet
    {
    private:
        // Subscriptions
        swri::Subscriber vel_msg_subscriber_;
        swri::Subscriber abs_pos_msg_subscriber_;

        // Publishers
        // Primary publishers
        ros::Publisher near_field_odom_publisher_;
        ros::Publisher far_field_odom_publisher_;
        ros::Publisher utm_odom_publisher_;
        ros::Publisher gps_utm_odom_publisher_;
        ros::Publisher utm_zone_pub_;
        ros::Publisher utm_band_pub_;

        // Transform Listener
        tf::TransformListener tf_listener_;
        // Pose Transformer
        sumet_state_estimator::PoseTransformer pose_transformer_;
        // Transform broadcaster
        tf::TransformBroadcaster Tf_Broadcaster_;

        // Timer to drive the output
        ros::Timer output_timer_;
        // Timer to delay initialization
        ros::WallTimer init_timer_;

        // Transform utils
        swri_transform_util::TransformManager tf_manager_;
        swri_transform_util::LocalXyWgs84UtilPtr local_xy_;
        swri_transform_util::UtmUtil utm_util_;
        uint8_t utm_zone_;
        char utm_band_;

        // Stored Transforms
        /**
         * TF from nf frame to vehicle base_link, based on relative
         * localization
         */
        tf::StampedTransform Rel_LocalXY_tf_;
        /**
         * TF from local xy (ff) to vehicle base_link, based on absolute
         * localization
         */
        tf::StampedTransform Abs_LocalXY_tf_;
        /// TF from local xy (ff) to nf frame
        tf::StampedTransform Abs_To_Rel_tf_;

        // Last Odometry
        nav_msgs::OdometryPtr last_far_field_odom_;
        nav_msgs::OdometryPtr last_utm_odom_;

        double dT_;
        double fs_;
        bool verbose_;

        ros::Time LastTime_;
        ros::Time last_pitch_and_roll_update_;

        // Field ids (stored as variables in order to modify with parameters
        /// Near field frame ID (default /near_field)
        std::string nf_id_;
        /// Vehicle near field frame ID (default /veh_near_field)
        std::string vehicle_frame_id_;
        std::string ff_id_;
        std::string gps_frame_;

        // UTM angle
        double utm_angle_;

        // Full State Estimator Localization Queue
        sumet_state_estimator::LocalizationQueue6DOF LQ_;

        // Relative Localization Queue -- For control
        sumet_state_estimator::LocalizationQueue6DOF RLQ_;

        // Diagnostic variables
        ros::Time last_gps_update_;
        ros::Time last_velocity_update_;
        boost::shared_ptr<diagnostic_updater::Updater> diagnostic_updater_;
        bool publish_diagnostics_;
        ros::Timer diagnostic_timer_;

        double gps_level1_timeout_;
        double gps_level2_timeout_;
        double published_count_;
        ros::Time last_diagnostic_update_;
        double initialization_heading_deviation_in_degrees_;
        double initialization_heading_error_thresh_in_degrees_;
        bool state_estimator_initialized_;

        bool do_fast_initialization_;
        bool fast_initialized_;
        double fast_init_variance_thresh_;

        boost::circular_buffer<OdomPair> odom_pairs_;
        // initialization_heading_check_status_:
        // -1: initializing
        //  0: initialized
        //  1: variance and measurement disagreement (waiting for re-initialization)
        int32_t initialization_heading_check_status_;

        void onInit()
        {
            double initialization_delay = 1.0;
            swri::param(getPrivateNodeHandle(),"initialization_delay_s", initialization_delay, 1.0);
            init_timer_ = getNodeHandle().createWallTimer(ros::WallDuration(initialization_delay),
                                                          &StateEst3DClass::initialize,
                                                          this,
                                                          true);
        }

        void initialize(const ros::WallTimerEvent&)
        {
            diagnostic_updater_ = boost::make_shared<diagnostic_updater::Updater>(
                getNodeHandle(), getPrivateNodeHandle(), getName());

            get_parameters();

            init_node();

            // Publish topics
            publish_topics();

            // Subscribe to topics
            subscribe_to_topics();

            setup_output_timer();
        }

        /**
         * @brief       Main timer triggered callback
         *
         * This function is where the output messages are computed and published
         * from. It is important that this node operate at a relatively constant
         * frequency.
         *
         * @param[in]   e    timer event data (automatically passed as part of the
         *                   callback

         */
        void localizationOutputCallback(const ros::TimerEvent& e)
        {
            ros::Time curTime = ros::Time::now();

            bool mode_process_until_last_timestamp = true;

            compute_and_publish_relative_location(curTime,
                                                  mode_process_until_last_timestamp);

            compute_and_publish_absolute_location(curTime,
                                                  mode_process_until_last_timestamp);

            publish_transforms();

            // Update diagnostic
            published_count_++;
        }

        void setup_output_timer()
        {
            ros::NodeHandle& pnh = getPrivateNodeHandle();
            output_timer_ = pnh.createTimer(
                ros::Duration(1.0 / fs_),
                &StateEst3DClass::localizationOutputCallback,
                this);
        }

        /**
         * @brief Computes any remaining transforms and publishes all transforms
         *      \dot
         *        digraph TFs {
         *        utm [ label="utm"];
         *        far_field [ label="far_field"];
         *        near_field [ label="near_field"];
         *        veh_near_field [ label="veh_near_field"];
         *        utm -> far_field [ arrowhead="open", style="solid"];
         *        far_field -> near_field [ arrowhead="open", style="solid" label="Abs_To_Rel_tf_"];
         *        near_field -> veh_near_field [ arrowhead="open", style="solid"  label="Rel_LocalXY_tf_"];
         *        }
         *      \enddot
         */
        void publish_transforms()
        {
            //////////////////////////////////////////////////////////////////////
            //                           UTM                                    //
            //                            |                                     //
            //                     far_field (swri)                             //
            //                            | (Abs_To_Rel_tf_)                    //
            //                       near_field                                 //
            //                            | (Rel_LocalXY_tf_)                   //
            //                      veh_near_field                              //
            //////////////////////////////////////////////////////////////////////

            // Send nf->vehicle transform
            Tf_Broadcaster_.sendTransform(Rel_LocalXY_tf_);

            // Compute the relative transform (ff->nf) by multiplying the
            // ff->vehicle transform by the inverse of the nf->vehicle transform
            Abs_To_Rel_tf_.stamp_ = Rel_LocalXY_tf_.stamp_;

            // Negate roll and pitch differences
            tf::Transform ff_to_nf;
            ff_to_nf = Abs_LocalXY_tf_ * Rel_LocalXY_tf_.inverse();
            
            double r;
            double p;
            double yaw;
            ff_to_nf.getBasis().getRPY(r, p, yaw);
            tf::Quaternion rpy;
            rpy.setRPY(0.0, 0.0, yaw);
            tf::Transform rot(rpy);
            tf::Vector3 translation = Abs_LocalXY_tf_.getOrigin() - rot*Rel_LocalXY_tf_.getOrigin();
            ff_to_nf.setRotation(rpy);
            ff_to_nf.setOrigin(translation);
            tf::StampedTransform ff_to_nf_stamped(ff_to_nf,
                                                  Rel_LocalXY_tf_.stamp_,
                                                  ff_id_,
                                                  nf_id_);
 

            // Send ff->nf transform
            Tf_Broadcaster_.sendTransform(ff_to_nf_stamped);
        }

        /**
         * @brief      Computes relative location, computes transform and publishes
         *             odometry
         *
         * @param[in]  curTime                           The time at which to estimate
         *                                               the position
         * @param[in]  process_until_last_timestamp      Don't predict past last
         *                                               timestamp if true
         */
        void compute_and_publish_relative_location(
            const ros::Time& curTime,
            bool process_until_last_timestamp)
        {
            // First we'll process the relative localization queue
            RLQ_.process_queue(curTime,
                               process_until_last_timestamp);

            nav_msgs::OdometryPtr rel_local_xy_odom(new nav_msgs::Odometry());
            tf::Transform RelTF;
            get_last_localXY(RLQ_,
                             *rel_local_xy_odom,
                             RelTF);

            rel_local_xy_odom->header.frame_id = nf_id_;
            near_field_odom_publisher_.publish(rel_local_xy_odom);
            Rel_LocalXY_tf_.setData(RelTF);
            Rel_LocalXY_tf_.stamp_ = rel_local_xy_odom->header.stamp;
        }

        /**
         * @brief      Computes absolute location, computes transform and publishes
         *             odometry
         *
         * @param[in]  curTime    The time at which to estimate the position
         * @param[in]  process_until_last_timestamp     Don't predict past last
         *                                              timestamp if true
         */
        void compute_and_publish_absolute_location(
            const ros::Time& curTime,
            bool process_until_last_timestamp)
        {
            // Process the current localization queues to estimate the current planar
            // state
            LQ_.process_queue(curTime, process_until_last_timestamp);

            // Get local XY odometry
            nav_msgs::OdometryPtr local_xy_odom(new nav_msgs::Odometry());
            tf::Transform AbsTF;
            get_last_localXY(LQ_, *local_xy_odom, AbsTF);

            Abs_LocalXY_tf_.setData(AbsTF);
            Abs_LocalXY_tf_.stamp_ = local_xy_odom->header.stamp;

            local_xy_odom->header.frame_id = ff_id_;
            far_field_odom_publisher_.publish(local_xy_odom);

            if (local_xy_->Initialized())
            {
                nav_msgs::OdometryPtr utm_odom(new nav_msgs::Odometry());
                uint8_t utm_zone;
                char utm_band;
                get_last_odom_UTM(LQ_, *utm_odom, utm_zone, utm_band);

                utm_odom_publisher_.publish(utm_odom);

                last_far_field_odom_ = local_xy_odom;
                last_utm_odom_ = utm_odom;
                // Only publish the UTM zone if it changes
                if (utm_zone_ != utm_zone)
                {
                    utm_zone_ = utm_zone;
                    std_msgs::UInt8Ptr msg(new std_msgs::UInt8());
                    msg->data = utm_zone_;
                    utm_zone_pub_.publish(msg);
                }
                // Only publish the UTM zone if it changes
                if (utm_band_ != utm_band)
                {
                    utm_band_ = utm_band;
                    std_msgs::CharPtr msg(new std_msgs::Char());
                    msg->data = utm_band_;
                    utm_band_pub_.publish(msg);
                }
            }
            else
            {
                NODELET_ERROR_THROTTLE(5.0, "State estimator waiting for local origin.");
            }
        }

        /**
         * @brief      Gets the last position update (since process_queue was last
         *             called on LQ_local)
         *
         * @param[in]  LQ_local      The localization queue to extract the odometry
         *                           from
         * @param[out] odom_out      The output odometry message
         * @param[out] trans_out     The transform corresponding to the odometry
         *                           message
         */
        void get_last_localXY(
            const sumet_state_estimator::LocalizationQueue6DOF& LQ_local,
            nav_msgs::Odometry& odom_out,
            tf::Transform& trans_out)
        {
            geometry_msgs::PoseWithCovarianceStamped AbsolutePose;
            LQ_local.getVehiclePose(AbsolutePose);
            odom_out.header.frame_id = ff_id_;
            odom_out.header.stamp = AbsolutePose.header.stamp;

            geometry_msgs::TwistWithCovarianceStamped VehicleTwist;
            LQ_local.getVehicleTwist(VehicleTwist);

            odom_out.pose = AbsolutePose.pose;
            odom_out.twist = VehicleTwist.twist;

            tf::Vector3 pos(AbsolutePose.pose.pose.position.x,
                            AbsolutePose.pose.pose.position.y,
                            AbsolutePose.pose.pose.position.z);

            tf::Quaternion Q(AbsolutePose.pose.pose.orientation.x,
                             AbsolutePose.pose.pose.orientation.y,
                             AbsolutePose.pose.pose.orientation.z,
                             AbsolutePose.pose.pose.orientation.w);

            trans_out.setOrigin(pos);
            trans_out.setRotation(Q);
        }

        /**
         * @brief      Converts VehiclePlanarState message into an odometry message
         *             in the /utm frame
         *
         * @param[in]   LQ_local              The localization queue to get the
         *                                    odometry from
         * @param[out]  odom_out              Converted odometry message
         */
        void get_last_odom_UTM(
            const sumet_state_estimator::LocalizationQueue6DOF& LQ_local,
            nav_msgs::Odometry& odom_out,
            uint8_t& utm_zone,
            char& utm_band)
        {
            /* TODO(kkozak): We're taking some liberties with the coordinate
             *      transformation here by equating directions between localxy
             *      and utm... we should make sure that the impact is small
             *      or do a full transformation (i.e. determine the
             *      transformation between the localxy and utm frame locally
             *      and use that to transform everything). Or we can just
             *      publish everything in LocalXY and let the transforms
             *      handle it. */

            // Extract the current estimated state
            geometry_msgs::PoseWithCovarianceStamped AbsolutePose;
            LQ_local.getVehiclePose(AbsolutePose);

            /* Set the header information -- note that it's best to get time
             * from the vehicle pose (rather than from the twist). */
            odom_out.header.frame_id = "/utm";
            odom_out.header.stamp = AbsolutePose.header.stamp;

            // Convert the pose to UTM
            geometry_msgs::PoseWithCovarianceStamped UtmPose;

            /* TODO(kkozak): If we copy like this the covariances and timestamps
             *      get copied over directly, but it should be noted that although
             *      the covariance matrices should be close, the coordinate
             *      transformation will affect it slightly. */
            UtmPose = AbsolutePose;

            convert_localxy_pose_to_utm_pose(AbsolutePose.pose.pose,
                                             UtmPose.pose.pose,
                                             utm_zone,
                                             utm_band);

            geometry_msgs::TwistWithCovarianceStamped VehicleTwist;
            LQ_local.getVehicleTwist(VehicleTwist);

            odom_out.pose = UtmPose.pose;
            odom_out.twist = VehicleTwist.twist;
        }

        /**
         * @brief      Converts the pose message in the localxy frame to a pose
         *             message in the utm frame
         *
         * @param[in]  LocalXyPose    The pose in Local XY
         * @param[out] UtmPose        The pose in UTM
         */
        void convert_localxy_pose_to_utm_pose(
            const geometry_msgs::Pose& LocalXyPose,
            geometry_msgs::Pose& UtmPose,
            uint8_t& utm_zone,
            char& utm_band)
        {
            // Convert from local_xy to wgs_84 first:
            double lat;
            double lon;
            local_xy_->ToWgs84(LocalXyPose.position.x,
                               LocalXyPose.position.y,
                               lat,
                               lon);

            // Convert from wgs_84 to UTM:
            double easting;
            double northing;
            int zone;
            utm_util_.ToUtm(lat, lon, zone, utm_band, easting, northing);

            UtmPose.position.x = easting;
            UtmPose.position.y = northing;
            utm_zone = static_cast<uint8_t>(zone);
        }

        /**
         * @brief      Gets the UTM zone for the current LocalXyPose
         *
         * @param[in]  LocalXyPose    The pose in Local XY
         * @param[out] UtmZone        The UTM zone of the pose
         */
        uint8_t GetUTMZone(
            const geometry_msgs::Pose& LocalXyPose,
            uint8_t& UtmZone)
        {
            // Convert from local_xy to wgs_84 first:
            double lat;
            double lon;
            local_xy_->ToWgs84(LocalXyPose.position.x,
                               LocalXyPose.position.y,
                               lat,
                               lon);
            return static_cast<uint8_t>(swri_transform_util::GetZone(lon));
        }

        /**
         * @brief      Aggregated velocity message callback
         *
         * @param[in]  msg    Twist message
         */
        void Velocity_cb(
            const geometry_msgs::TwistWithCovarianceStampedConstPtr& msg)
        {
            check_validity(msg);
            sumet_state_estimator::Velocity3DLocalizationElement Temp(*msg);
            LQ_.insert_velocity_element(Temp);

            sumet_state_estimator::Velocity3DLocalizationElement Temp1(*msg);
            RLQ_.insert_velocity_element(Temp1);
        }

        /**
         * @brief      Checks the validity of the velocity message and prints an
         *             error if covariances are above 1000.0
         *
         * @param[in]  msg    Twist message
         */
        void check_validity(
            const geometry_msgs::TwistWithCovarianceStampedConstPtr& msg)
        {
            const boost::array<double, 36>& cov = msg->twist.covariance;

            int i = 0;
            if (std::abs(cov[i * i + i]) > 1000.0)
            {
                NODELET_ERROR("Linear velocity not valid: val = %g, cov = %g",
                          msg->twist.twist.linear.x,
                          cov[i * i + i]);
            }
            i = 3;
            if (std::abs(cov[i * i + i]) > 1000.0)
            {
                NODELET_ERROR("wx not valid: val = %g, cov = %g",
                          msg->twist.twist.angular.x,
                          cov[i * i + i]);
            }
            i = 4;
            if (std::abs(cov[i * i + i]) > 1000.0)
            {
                NODELET_ERROR("wy not valid: val = %g, cov = %g",
                          msg->twist.twist.angular.y,
                          cov[i * i + i]);
            }
            i = 5;
            if (std::abs(cov[i * i + i]) > 1000.0)
            {
                NODELET_ERROR("wz not valid: val = %g, cov = %g",
                          msg->twist.twist.angular.z,
                          cov[i * i + i]);
            }
        }

        /**
         * @brief      Callback for absolute position updates
         *
         * Transforms GPS position updates into the vehicle near field frame.
         *
         * @param[in]  msg    The pose message from the
         *                    sumet_3D_position_aggregator_node
         */
        void Position_cb(
            const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
        {
            // Check to see whether this message was generated with GPS data
            if (is_gps_measurement(msg))
            {
                NODELET_DEBUG("Got a GPS position message.");
                bool needs_copy = false;
                // This method is called very frequently, and in some cases we need to make
                // a duplicate of the message and modify it, but not all.  A good optimization
                // here is to avoid copying the message if we can.  So, we do a few
                // checks on it and make a copy and modify it if necessary...
                geometry_msgs::PoseWithCovarianceStampedPtr new_msg;
                if (do_fast_initialization_ && !fast_initialized_)
                {
                    NODELET_DEBUG("Estimator has not been fast-initialized.");
                    if (msg->pose.covariance[21] < fast_init_variance_thresh_)
                    {
                        fast_initialized_ = true;
                        NODELET_DEBUG("Fast-initialized estimator");
                    }
                    else
                    {
                        NODELET_DEBUG("Cannot fast-init estimator. Yaw variance too high (%f)",
                                  msg->pose.covariance[21]);
                        needs_copy = true;
                        new_msg.reset(new geometry_msgs::PoseWithCovarianceStamped(*msg));
                        new_msg->pose.covariance[21] = 1e20;
                    }
                }
                // and then we initialize a reference based on whether we can use the
                // original or not.
                const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg_tmp = needs_copy ? new_msg : msg;
                publish_gps(msg);

                // Added this part to support enhanced measurement of initialization
                // status
                OdomPair cur_pair;
                nav_msgs::OdometryPtr gps_odom(new nav_msgs::Odometry());
                gps_odom->header.stamp = msg->header.stamp;
                gps_odom->pose = msg->pose;
                cur_pair.ff_odom = last_far_field_odom_;
                cur_pair.gps_odom = gps_odom;
                odom_pairs_.push_back(cur_pair);

                // Diagnostic variable update
                last_gps_update_ = ros::Time::now();

                sumet_state_estimator::Absolute3DLocalizationElement AbsPosition;
                // Use msg_temp in case the fast_initialization is being used.
                geometry_msgs::PoseWithCovarianceStampedPtr msg_veh =
                    boost::make_shared<geometry_msgs::PoseWithCovarianceStamped>();

                // Convert message into vehicle frame
                pose_transformer_.transform_pose(gps_frame_, vehicle_frame_id_,
                                                 msg_tmp, msg_veh,
                                                 sqrt(LQ_.GetCovariance()(5,5)));

                AbsPosition.load_pose_data_from_msg(*msg_veh);

                LQ_.insert_absolute_position_element(AbsPosition);
            }
            else
            {
                NODELET_DEBUG("Got a non-GPS position message with frame_id %s.", msg->header.frame_id.c_str());
                sumet_state_estimator::Absolute3DLocalizationElement PitchAndRollUpdate;
                PitchAndRollUpdate.load_pose_data_from_msg(*msg);

                RLQ_.insert_absolute_position_element(PitchAndRollUpdate);

                sumet_state_estimator::Absolute3DLocalizationElement AbsPosition;
                AbsPosition.load_pose_data_from_msg(*msg);

                LQ_.insert_absolute_position_element(AbsPosition);
            }
        }

        /**
         * @brief      Publishes a GPS odometry message in the UTM frame for
         *             visualization
         *
         * @param[in]  msg    A Pose message that contains GPS information
         */
        void publish_gps(
            const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
        {
            if (local_xy_->Initialized())
            {
                // Extract the current estimated state
                geometry_msgs::PoseWithCovarianceStamped AbsolutePose = *msg;

                nav_msgs::OdometryPtr odom_out(new nav_msgs::Odometry());
                odom_out->header.frame_id = "/utm";
                odom_out->header.stamp = AbsolutePose.header.stamp;

                // Convert the pose to UTM
                geometry_msgs::PoseWithCovarianceStamped UtmPose;

                // TODO(kkozak): If we copy like this the covariances and timestamps get
                // copied over directly, but it should be noted that although the covariance
                // matrices should be close, the coordinate transformation will affect it
                // slightly
                UtmPose = AbsolutePose;
                uint8_t zone;
                char band;
                convert_localxy_pose_to_utm_pose(AbsolutePose.pose.pose,
                                                 UtmPose.pose.pose,
                                                 zone,
                                                 band);

                odom_out->pose = UtmPose.pose;

                gps_utm_odom_publisher_.publish(odom_out);
            }
            else
            {
                NODELET_ERROR("State estimator waiting for local origin");
            }
        }

        /**
         * @brief      Determines whether measurement includes GPS data in it
         *
         * @param[in]  msg      The Pose message to test
         *
         * @retval     Returns true if the message contains GPS data
         */
        bool is_gps_measurement(
            const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
        {
            std::string pattern("gps");
            std::string str(msg->header.frame_id);
            if (str.find(pattern) != std::string::npos)
            {
                return true;
            }

            return false;
        }

        /**
         * @brief      Subscribe to topics
         */
        void subscribe_to_topics()
        {
            ros::NodeHandle& nh = getNodeHandle();
            vel_msg_subscriber_ = swri::Subscriber(nh,
                                                   "VehicleVelocity6D",
                                                   100,
                                                   &StateEst3DClass::Velocity_cb,
                                                   this);

            abs_pos_msg_subscriber_ = swri::Subscriber(nh,
                                                       "AbsolutePoseMeasurement",
                                                       100,
                                                       &StateEst3DClass::Position_cb,
                                                       this);
        }

        /**
         * @brief      Advertise topic names
         */
        void publish_topics()
        {
            ros::NodeHandle& nh = getNodeHandle();
            near_field_odom_publisher_ = swri::advertise<nav_msgs::Odometry>(nh,
                "near_field_odom", 100);

            utm_odom_publisher_ = swri::advertise<nav_msgs::Odometry>(nh,
                "utm_odom", 100);

            far_field_odom_publisher_ = swri::advertise<nav_msgs::Odometry>(nh,
                "far_field_odom", 100);

            gps_utm_odom_publisher_ = swri::advertise<nav_msgs::Odometry>(nh,
                "gps_utm_odom", 100);

            utm_zone_pub_ = swri::advertise<std_msgs::UInt8>(nh,
                "utm_zone", 10, true);

            utm_band_pub_ = swri::advertise<std_msgs::Char>(nh,
                "utm_band", 10, true);
        }

        /**
         * @brief       Contains all of the miscellaneous initialization steps for
         *              the node
         */
        void init_node()
        {
            init_diagnostics();

            local_xy_ = boost::make_shared<swri_transform_util::LocalXyWgs84Util>();

            // Set up Pose Transformer
            pose_transformer_.setWorldFrame(ff_id_);

            // Set up TF tree
            Rel_LocalXY_tf_.frame_id_ = nf_id_;
            Rel_LocalXY_tf_.child_frame_id_ = vehicle_frame_id_;

            Abs_LocalXY_tf_.frame_id_ = ff_id_;
            Abs_LocalXY_tf_.child_frame_id_ = vehicle_frame_id_;

            Abs_To_Rel_tf_.frame_id_ = ff_id_;
            Abs_To_Rel_tf_.child_frame_id_ = nf_id_;

            // Initialize diagnostics variables
            last_gps_update_ = ros::TIME_MIN;
            last_velocity_update_ = ros::TIME_MIN;
            last_diagnostic_update_ = ros::TIME_MIN;
            published_count_ = 0;

            odom_pairs_.set_capacity(50);
            initialization_heading_check_status_ = -1;
        }

        void get_parameters()
        {
            ros::NodeHandle& pnh = getPrivateNodeHandle();
            // TODO: The following line should not have a leading slash (this looks in
            // the global namespace, but I (evenator) don't want to change it because
            // it might break something.
            swri::param(pnh,"/sampling_frequency", fs_, DEFAULT_SAMPLING_FREQUENCY);

            // Initialize sampling time
            dT_ = 1 / fs_;

            swri::param(pnh,"publish_diagnostics", publish_diagnostics_, true);
            swri::param(pnh,"gps_level1_timeout", gps_level1_timeout_, 0.2);
            swri::param(pnh,"gps_level2_timeout", gps_level2_timeout_, 5.0);
            swri::param(pnh,"initialization_heading_deviation_in_degrees",
                      initialization_heading_deviation_in_degrees_,
                      0.5);
            // this was added to allow for a direct check or ff_odom heading against
            // ongoing gps measurements.  The threshold should be higher for this
            // one than for the deviation above.
            swri::param(pnh,"initialization_heading_error_thresh_in_degrees",
                      initialization_heading_error_thresh_in_degrees_,
                      2.0);

            swri::param(pnh,"verbose", verbose_, false);
            swri::param(pnh,"near_field_id", nf_id_, std::string("/near_field"));
            swri::param(pnh,"near_field_veh_id", vehicle_frame_id_, std::string("/veh_near_field"));
            swri::param(pnh,"far_field_id", ff_id_, std::string("/far_field"));
            swri::param(pnh,"gps_frame_id", gps_frame_, std::string("/gps"));

            swri::param(pnh,"do_fast_initialization", do_fast_initialization_, false);
            swri::param(pnh,"fast_init_variance_thresh", fast_init_variance_thresh_, 10.0);

            LQ_.set_verbose(verbose_);
            RLQ_.set_verbose(verbose_);
        }

        void init_diagnostics()
        {
            if (!publish_diagnostics_)
            {
                return;
            }

            // Setup diagnostics.
            diagnostic_updater_->setHardwareID("state_estimator");

            diagnostic_updater_->add("GPS Status",
                                    this,
                                    &StateEst3DClass::GpsDiagnostic);

            diagnostic_updater_->add("Frequency Status",
                                    this,
                                    &StateEst3DClass::FrequencyDiagnostic);

            diagnostic_updater_->add("Initialization Status",
                                    this,
                                    &StateEst3DClass::InitializationDiagnostic);

            diagnostic_updater_->add("Communications",
                                    this,
                                    &StateEst3DClass::communicationDiagnostic);

            ros::NodeHandle& nh = getNodeHandle();
            diagnostic_timer_ =
                nh.createTimer(ros::Duration(1),
                                &StateEst3DClass::RunDiagnostics,
                                this);
        }

        /**
         * @brief      Diagnostic on the output rate of the state estimator
         *
         * @param status
         */
        void FrequencyDiagnostic(
            diagnostic_updater::DiagnosticStatusWrapper& status)
        {
            double count = published_count_;
            double period = diagnostic_updater_->getPeriod();
            ros::Time now = ros::Time::now();
            if (last_diagnostic_update_ != ros::TIME_MIN)
            {
                period = (now - last_diagnostic_update_).toSec();
            }
            last_diagnostic_update_ = now;

            double freq = count / period;
            if (freq < fs_ * 0.8)
            {
                status.summary(status.WARN, "Desired frequency not met");
            }
            else
            {
                status.summary(status.OK, "Desired frequency met");
            }

            status.add("Expected frequency", fs_);
            status.add("Actual frequency", freq);
        }

        /**
         * @brief Diagnostic on the GPS availability at the state estimator node
         *
         * @param status
         */
        void GpsDiagnostic(diagnostic_updater::DiagnosticStatusWrapper& status) // NOLINT
        {
            ros::Time cur_time = ros::Time::now();
            double time_since_last_gps_update =
                (cur_time - last_gps_update_).toSec();

            if (time_since_last_gps_update < gps_level1_timeout_)
            {
                status.summary(status.OK, "GPS is available and active in state estimator");
            }
            else if (time_since_last_gps_update < gps_level2_timeout_)
            {
                char buff[1024];
                snprintf(buff, sizeof(buff), "Last GPS update %f seconds ago",
                         time_since_last_gps_update);
                status.summary(status.WARN, std::string(buff));
            }
            else
            {
                char buff[1024];
                snprintf(buff, sizeof(buff), "Last GPS update %f seconds ago",
                         time_since_last_gps_update);
                status.summary(status.ERROR, std::string(buff));
            }
            status.add("Time since last GPS Update", time_since_last_gps_update);
        }

        /**
         * @brief      Reports the status of the system with respect to whether the
         *             the system has been adequately initialized since startup.
         *
         * @param      status
         */
        void InitializationDiagnostic(
            diagnostic_updater::DiagnosticStatusWrapper& status)
        {
            // Here we use the heading variance as an indicator of
            // initialization status.
            double heading_variance = LQ_.getHeadingVariance();
            double heading_deviation_deg =
                sumet_util::MathUtil::ToDegrees(std::sqrt(heading_variance));

            double hme = HeadingMeasurmentError();
            if (heading_deviation_deg > initialization_heading_deviation_in_degrees_)
            {
                initialization_heading_check_status_ = -1;
                state_estimator_initialized_ = false;
                status.summary(status.WARN, "State estimator is not yet initialized");
            }
            else
            {
                // Check the heading measurement error:
                if (hme >= 0)
                {
                    if (hme > initialization_heading_error_thresh_in_degrees_)
                    {
                        initialization_heading_check_status_ = 1;
                        state_estimator_initialized_ = false;
                        status.summary(status.WARN, "State estimator is not initialized");
                    }
                    else
                    {
                        initialization_heading_check_status_ = 0;
                        state_estimator_initialized_ = true;
                        status.summary(status.OK, "State estimator is initialized");
                    }
                }
                else
                {
                    if (initialization_heading_check_status_ <= 0)
                    {
                        state_estimator_initialized_ = true;
                        status.summary(status.OK, "State estimator is initialized");
                    }
                    else
                    {
                        state_estimator_initialized_ = false;
                        status.summary(status.WARN, "State estimator is not initialized");
                    }
                }
            }

            status.add("Heading deviation (Deg)", heading_deviation_deg);
        }

        void communicationDiagnostic(diagnostic_updater::DiagnosticStatusWrapper& status)
        {
            status.summary(diagnostic_updater::DiagnosticStatusWrapper::OK, "No errors reported.");
            vel_msg_subscriber_.appendDiagnostics(status, "Vehicle Velocity",
                                                  swri::Subscriber::DIAG_MOST);
            abs_pos_msg_subscriber_.appendDiagnostics(status, "Absolute Pose Measurement",
                                                      swri::Subscriber::DIAG_MOST);
        }

        /**
         * @brief      Computes a nominal heading error (between GPS and ff_odom)
         *
         * @retval     Returns -1.0 if odom_pairs_ is stale, or no valid odometry is
         *             available in the buffer, returns RMSE otherwise
         */
        double HeadingMeasurmentError(
            double staleness_time_thresh = 5.0,
            int32_t min_num_valid_samples = 5)
        {
            double val = -1.0;

            ros::Time now = ros::Time::now();
            int32_t num_valid_samples = 0;
            double sum = 0.0;
            for (size_t i = 0; i < odom_pairs_.size(); ++i)
            {
                // Check age:
                if ((now - odom_pairs_[i].ff_odom->header.stamp).toSec() > staleness_time_thresh ||
                    (now - odom_pairs_[i].gps_odom->header.stamp).toSec() > staleness_time_thresh)
                {
                    // Data is stale
                    continue;
                }
                else if (
                    std::abs((odom_pairs_[i].ff_odom->header.stamp - odom_pairs_[i].gps_odom->header.stamp).toSec()) >
                    0.1)
                {
                    // GPS and far_field odom is out of sync
                    continue;
                }
                else if (odom_pairs_[i].ff_odom->twist.twist.linear.x < 1.0)
                {
                    // Velocity is too slow to do a useful comparison
                    continue;
                }
                else if (odom_pairs_[i].gps_odom->pose.covariance[0] > 100.0)
                {
                    // GPS error is too high for this sample
                    continue;
                }

                num_valid_samples++;
                tf::Quaternion qff(
                    odom_pairs_[i].ff_odom->pose.pose.orientation.x,
                    odom_pairs_[i].ff_odom->pose.pose.orientation.y,
                    odom_pairs_[i].ff_odom->pose.pose.orientation.z,
                    odom_pairs_[i].ff_odom->pose.pose.orientation.w);
                tf::Quaternion qgps(
                    odom_pairs_[i].gps_odom->pose.pose.orientation.x,
                    odom_pairs_[i].gps_odom->pose.pose.orientation.y,
                    odom_pairs_[i].gps_odom->pose.pose.orientation.z,
                    odom_pairs_[i].gps_odom->pose.pose.orientation.w);

                double roll;
                double pitch;
                double yaw_odom;
                double yaw_gps;
                tf::Transform(qff).getBasis().getRPY(roll, pitch, yaw_odom);
                tf::Transform(qgps).getBasis().getRPY(roll, pitch, yaw_gps);
                yaw_gps = sumet_util::MathUtil::WrapAngle(yaw_odom, yaw_gps);
                double diff = yaw_gps - yaw_odom;
                sum += std::pow(diff, 2.0);
            }

            if (num_valid_samples >= min_num_valid_samples)
            {
                val = std::sqrt(sum / static_cast<double>(num_valid_samples));
            }
            return val;
        }

        /**
         * @brief      The diagnostic updater function
         * @param e
         */
        void RunDiagnostics(const ros::TimerEvent& e)
        {
            diagnostic_updater_->force_update();
            published_count_ = 0;
        }


    public:
        /**
         * @brief        Constructor
         */
        explicit StateEst3DClass() :
            tf_listener_(ros::Duration(30.0)),
            last_far_field_odom_(new nav_msgs::Odometry()),
            last_utm_odom_(new nav_msgs::Odometry()),
            fs_(DEFAULT_SAMPLING_FREQUENCY),
            state_estimator_initialized_(false),
            do_fast_initialization_(false),
            fast_initialized_(false),
            utm_zone_(61), //invalid initial value to guarantee it is published
            utm_band_('A') //invalid initial value to guarantee it is published
        {
        }

        ~StateEst3DClass()
        {
        }
    };
};

// Register nodelet plugin
#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(sumet_state_estimator,
                        sumet_3D_state_estimator_nodelet,
                        sumet_state_estimator::StateEst3DClass,
                        nodelet::Nodelet)
