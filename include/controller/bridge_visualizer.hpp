// -*- lots-wrapper-c++ -*-

/* ---------------------------------------------------------------------------------------------------------------------
 * Package: LOTS-WRAPPER
 * Class: BridgeVisualizer
 * Author: Mirko Usuelli
 * Advisor: Prof. Matteo Matteucci, PhD
 * Co-Advisors: Matteo Frosi, PhD; Gianluca Bardaro, PhD; Simone Mentasti, PhD; Paolo Cudrano, PhD Student.
 * University: Politecnico di Milano - Artificial Intelligence & Robotics Lab
 * ---------------------------------------------------------------------------------------------------------------------
 * This file is part of {{ LOTS-WRAPPER }}.
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

#ifndef WRAPPER_BRIDGE_VISUALIZER_H
#define WRAPPER_BRIDGE_VISUALIZER_H

// ARTSLAM libraries
#include <lots_utils/dispatcher.h>

// Observers libraries
#include "observers.h"

// ROS libraries
#include "rclcpp/rclcpp.hpp"

// TFs libraries
#include "tf2_ros/buffer.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/buffer_interface.h"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace lots::slam::wrapper {
    using namespace lots::core::types;
/**
 * ARTSLAMBridgeVisualizer
 *
 * Class module in charge of interacting with ARTSLAM observers with respect to the SLAM-Output
 * (i.e. Map and Poses) through the "SlamOutputObserver" and to the Filtered Pointcloud through the
 * "FilteredPointcloudObserver".
 * This interaction with the ARTSLAM architecture is due to the visual monitoring task of this class which
 * acts as a bridge between ARTSLAM and ROS through a Visualizer tool such as Rviz or Fox Glove Studio.
 */
    class BridgeVisualizer : public SLAMOutputObserver {
        // Constants
        const std::string MODULE_ID = "VisualizerBridgeDispatcher";
        const std::string TOPIC_ROOT = "/artslam_wrapper";
        const std::string POSE_TOPIC = TOPIC_ROOT + "/pose";
        const std::string POINTCLOUD_TOPIC = TOPIC_ROOT + "/pointcloud";
        const std::string MARKER_TOPIC = TOPIC_ROOT + "/marker";
        const std::string OCCUPANCYGRID_TOPIC = TOPIC_ROOT + "/occupancy_grid";
        const std::string ODOM_TOPIC = TOPIC_ROOT + "/odom";

    public:
        /* Attributes ----------------------------------------------------------------------------------- */
        // TFs
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
        std::unique_ptr<tf2_ros::Buffer> tf_buffer;

        std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
        uint delay;

        std::string base_frame;
        std::string odom_frame;
        std::string global_frame;

        /* Methods -------------------------------------------------------------------------------------- */
        // Constructor
        explicit BridgeVisualizer(std::shared_ptr<rclcpp::Node> n);

        void init_tf();

        // Observer updating interfaces
        void update_slam_output_observer(const SLAMOutput_MSG::Ptr& slam_output, const std::string& id) override;
        void update_slam_output_observer(const SLAMOutput_MSG::ConstPtr& slam_output, const std::string& id) override;

//        void update_point_cloud_observer(pcl::PointCloud<Point3I>::ConstPtr pointcloud) override;
//
//        void update_slam_output_observer(
//                pcl::PointCloud<Point3I>::Ptr map,
//                std::vector<Eigen::Isometry3d> poses,
//                OccupancyGrid_MSG::Ptr occupancy_grid) override;
//
//        void update_odometry_observer(const Odometry_MSG::ConstPtr &message) override;

    private:
        /* Attributes ----------------------------------------------------------------------------------- */
        // Core visualization dispatcher
        std::unique_ptr<lots::core::utils::Dispatcher> dispatcher;

        // ROS Node handler
        std::shared_ptr<rclcpp::Node> node;

        // ROS Publishers
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occgrid_map_pub;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
        rclcpp::TimerBase::SharedPtr tf_pub;

        // TF
        geometry_msgs::msg::TransformStamped latest_transform;

        /* Methods -------------------------------------------------------------------------------------- */
        // Drawing method to refresh rviz or foxglove-studio
        void draw_pointcloud(const pcl::PointCloud<Point3I>::ConstPtr &pointcloud);

        void draw_map_and_poses(pcl::PointCloud<Point3I>::Ptr map, std::vector<EigIsometry3d> poses,
                                OccupancyGrid_MSG::Ptr occupancy_grid);

        void timer_callback();
    };
}

#endif // WRAPPER_BRIDGE_VISUALIZER_H
