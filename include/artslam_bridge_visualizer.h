// -*- artslam-c++ -*-

/* ---------------------------------------------------------------------------------------------------------------------
 * Package: ARTSLAM-WRAPPER
 * Class: ARTSLAMBridgeVisualizer
 * Author: Mirko Usuelli
 * Advisor: Prof. Matteo Matteucci, PhD
 * Co-Advisors: Matteo Frosi, PhD; Gianluca Bardaro, PhD; Simone Mentasti, PhD; Paolo Cudrano, PhD Student.
 * University: Politecnico di Milano - Artificial Intelligence & Robotics Lab
 * Last Modified: 30/11/2022
 * ---------------------------------------------------------------------------------------------------------------------
 * This file is part of {{ ARTSLAM_WRAPPER }}.
 *
 * Developed for the Politecnico di Milano (Artificial Intelligence & Robotics Lab)
 * This product includes software developed by Matteo Frosi. See the README file at the top-level directory of this
 * distribution for details of code ownership.
 *
 * This program is free software: you can redistribute it and/or modify it.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * ---------------------------------------------------------------------------------------------------------------------
 */

#ifndef ARTSLAM_BRIDGE_VISUALIZER_H
#define ARTSLAM_BRIDGE_VISUALIZER_H

// ARTSLAM libraries
#include <artslam_utils/dispatcher.h>

// Observers libraries
#include <observers/pointcloud_observers.h>
#include <observers/output_observer.h>

// ROS libraries
#include <ros/ros.h>
#include <ros/publisher.h>

// TFs libraries
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>


namespace artslam
{
    namespace laser3d
    {
        /**
         * ARTSLAMBridgeVisualizer
         *
         * Class module in charge of interacting with ARTSLAM observers with respect to the SLAM-Output
         * (i.e. Map and Poses) through the "SlamOutputObserver" and to the Filtered Pointcloud through the
         * "FilteredPointcloudObserver".
         * This interaction with the ARTSLAM architecture is due to the visual monitoring task of this class which
         * acts as a bridge between ARTSLAM and ROS through a Visualizer tool such as Rviz or Fox Glove Studio.
         */
        class ARTSLAMBridgeVisualizer :
                public FilteredPointcloudObserver,
                public SlamOutputObserver {
            // Constants
            const std::string MODULE_ID = "VisualizerBridgeDispatcher";
            const std::string TOPIC_ROOT = "/artslam_wrapper";
            const std::string POSE_TOPIC = TOPIC_ROOT + "/pose";
            const std::string POINTCLOUD_TOPIC = TOPIC_ROOT + "/pointcloud";
            const std::string MARKER_TOPIC = TOPIC_ROOT + "/marker";

        public:
            /* Attributes ------------------------------------------------------------------------------------------- */
            // TFs
            tf2_ros::TransformBroadcaster tf_broadcaster;
            tf2_ros::Buffer tf_buffer;
            tf2_ros::TransformListener tf_listener;

            /* Methods ---------------------------------------------------------------------------------------------- */
            // Constructor
            ARTSLAMBridgeVisualizer();

            explicit ARTSLAMBridgeVisualizer(ros::NodeHandle &nh);

            // Setter
            void set_handler(ros::NodeHandle &nh) { handler = nh; };

            // Observer updating interfaces
            void update_filtered_pointcloud_observer(pcl::PointCloud<Point3I>::ConstPtr pointcloud) override;

            void update_slam_output_observer(
                    pcl::PointCloud<Point3I>::Ptr map,
                    std::vector <Eigen::Isometry3d> poses
            ) override;

        private:
            /* Attributes ------------------------------------------------------------------------------------------- */
            // Core visualization dispatcher
            std::unique_ptr <artslam::core::utils::Dispatcher> dispatcher;

            // ROS Node handler
            ros::NodeHandle handler;

            // ROS Publishers
            ros::Publisher markers_pub;
            ros::Publisher pose_pub;
            ros::Publisher pointcloud_pub;

            /* Methods ---------------------------------------------------------------------------------------------- */
            // Drawing method to refresh rviz or foxglove-studio
            void draw_pointcloud(const pcl::PointCloud<Point3I>::ConstPtr &pointcloud);

            void draw_map_and_poses(pcl::PointCloud<Point3I>::Ptr map, std::vector <EigIsometry3d> poses);

            // Matrix transformation util
            geometry_msgs::TransformStamped matrix2transform(
                    const ros::Time &stamp,
                    const Eigen::Matrix4f &pose,
                    const std::string &frame_id,
                    const std::string &child_frame_id
            );
        };
    }
}

#endif // ARTSLAM_BRIDGE_VISUALIZER_H
