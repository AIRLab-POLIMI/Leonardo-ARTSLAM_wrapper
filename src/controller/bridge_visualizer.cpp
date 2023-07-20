// -*- lots-wrapper-c++ -*-

/* ---------------------------------------------------------------------------------------------------------------------
 * Package: LOTS-WRAPPER
 * Class: BridgeVisualizer
 * Author: Mirko Usuelli
 * Advisor: Prof. Matteo Matteucci, PhD
 * Co-Advisors: Matteo Frosi, PhD; Gianluca Bardaro, PhD; Simone Mentasti, PhD;
 * Paolo Cudrano, PhD Student. University: Politecnico di Milano - Artificial
 * Intelligence & Robotics Lab
 * ---------------------------------------------------------------------------------------------------------------------
 * This file is part of {{ LOTS-WRAPPER }}.
 *
 * Developed for the Politecnico di Milano (Artificial Intelligence & Robotics
 * Lab) This product includes software developed by Matteo Frosi. See the README
 * file at the top-level directory of this distribution for details of code
 * ownership.
 *
 * This program is free software: you can redistribute it and/or modify it.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.
 * ---------------------------------------------------------------------------------------------------------------------
 */

// Header library
#include "controller/bridge_visualizer.hpp"

// ROS Messages libraries
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// TFs libraries
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer_interface.h>
#include <tf2/transform_datatypes.h>
#include <tf2/convert.h>

// PCL libraries
#include <pcl_conversions/pcl_conversions.h>
#include <chrono>
#include <rclcpp/duration.hpp>

using namespace std::chrono_literals;

namespace lots::slam::wrapper {
/**
 * Initialize the bridge-visualizer for ARTSLAM-ROS functionalities.
 */
    BridgeVisualizer::BridgeVisualizer(std::shared_ptr<rclcpp::Node> n) {
        node = n;

        tf_buffer = std::make_unique<tf2_ros::Buffer>(node->get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
        tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(node);

        // Dispatcher initialization for the visualizer
        dispatcher = std::make_unique<core::utils::Dispatcher>(MODULE_ID, 1);

        // ROS Publishers initialization
        markers_pub = node->create_publisher<visualization_msgs::msg::MarkerArray>(MARKER_TOPIC, 16);
        pointcloud_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>(POINTCLOUD_TOPIC, 1);
        pose_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(POSE_TOPIC, 10);
        occgrid_map_pub = node->create_publisher<nav_msgs::msg::OccupancyGrid>(OCCUPANCYGRID_TOPIC, 1);
        odom_pub = node->create_publisher<nav_msgs::msg::Odometry>(ODOM_TOPIC, 1);

        tf_pub = node->create_wall_timer(100ms, std::bind(&BridgeVisualizer::timer_callback, this));
    }

    void BridgeVisualizer::init_tf() {
        // Initial TF configuration message map->odom
        geometry_msgs::msg::TransformStamped init_msg;
        init_msg.header.stamp = node->now();
        init_msg.header.frame_id = global_frame;
        init_msg.child_frame_id = odom_frame;
        init_msg.transform.translation.x = init_msg.transform.translation.y = init_msg.transform.translation.z = 0;
        init_msg.transform.rotation.x = init_msg.transform.rotation.y = init_msg.transform.rotation.z = 0;
        init_msg.transform.rotation.w = 1;
        tf_broadcaster->sendTransform(init_msg);
    }

    void BridgeVisualizer::timer_callback() {
        if (!latest_transform.header.frame_id.empty()) {
            latest_transform.header.stamp = node->now() + rclcpp::Duration(0, delay);
            tf_broadcaster->sendTransform(latest_transform);
        }
    }

/**
 * It signals that a new filtered point cloud has been received and it is going
 * to be displayed.
 *
 * @param pointcloud Filtered PCL pointcloud to be displayed.
 */
//    void BridgeVisualizer::update_point_cloud_observer(pcl::PointCloud<Point3I>::ConstPtr pointcloud) {
//        // sending the point cloud to the visualizer through the dispatcher
//        dispatcher->dispatch([this, pointcloud] { draw_pointcloud(pointcloud); });
//    }

/**
 * It signals that map and poses are ready to be displayed.
 *
 * @param map Map to be displayed.
 * @param poses Poses to be displayed.
 */
//    void BridgeVisualizer::update_slam_output_observer(pcl::PointCloud<Point3I>::Ptr map,
//                                                       std::vector<Eigen::Isometry3d> poses,
//                                                       OccupancyGrid::Ptr occupancy_grid) {
//        // sending map and poses to the visualizer through the dispatcher
//        std::cout << "update_slam_output_observer" << std::endl;
//        dispatcher->dispatch([this, map, poses, occupancy_grid] {
//            draw_map_and_poses(map, poses, occupancy_grid);
//        });
//    }

/**
 * It draws a new received pointcloud.
 *
 * @param pointcloud Filtered PCL pointcloud to be displayed.
 */
    void BridgeVisualizer::draw_pointcloud(const pcl::PointCloud<Point3I>::ConstPtr &pointcloud) {
        // pointcloud ros message
        sensor_msgs::msg::PointCloud2 pointcloud_msg;

        // pointcloud pcl
        pcl::PointCloud<Point3I>::Ptr pointcloud_pcl(new pcl::PointCloud<Point3I>);

        // point cloud publishing
        *pointcloud_pcl = *pointcloud;
        pointcloud_pcl->header.stamp /= 1000ull;
        pcl::toROSMsg(*pointcloud_pcl, pointcloud_msg);
        pointcloud_msg.header.frame_id = base_frame;
        pointcloud_pub->publish(pointcloud_msg);
    }

    void BridgeVisualizer::draw_map_and_poses(pcl::PointCloud<Point3I>::Ptr map, std::vector<EigIsometry3d> poses,
                                              OccupancyGrid_MSG::Ptr occupancy_grid) {

    }

    void
    BridgeVisualizer::update_slam_output_observer(const SLAMOutput_MSG::Ptr &slam_output, const std::string &id) {
        //TODO call draw map and poses and draw point cloud

    }

    void
    BridgeVisualizer::update_slam_output_observer(const SLAMOutput_MSG::ConstPtr &slam_output, const std::string &id) {

    }

/**
 * Signals that map and poses are ready to be displayed.
 *
 * @param map Map to be displayed.
 * @param poses Poses to be displayed.
 */
//    void BridgeVisualizer::draw_map_and_poses(pcl::PointCloud<Point3I>::Ptr map, std::vector<EigIsometry3d> poses,
//                                              OccupancyGrid::Ptr occupancy_grid) {
//        std::cout << "drawing map" << std::endl;
//        // pointcloud message
//        sensor_msgs::msg::PointCloud2 pointcloud_msg;
//        rclcpp::Time stamp(map->header.stamp / 1000000000ull, map->header.stamp % 1000000000ull);
//        map->header.stamp /= 1000ull;
//        pcl::toROSMsg(*map, pointcloud_msg);
//        pointcloud_msg.header.frame_id = global_frame;
//        pointcloud_msg.header.stamp = stamp;
//        pointcloud_pub->publish(pointcloud_msg);
//
//        // occupancy grid message
//        nav_msgs::msg::OccupancyGrid occupancy_grid_msg;
//        occupancy_grid_msg.header.frame_id = global_frame;
//        occupancy_grid_msg.header.stamp = stamp;
//        occupancy_grid_msg.info.resolution = occupancy_grid->resolution_;
//        occupancy_grid_msg.info.width = occupancy_grid->width_;
//        occupancy_grid_msg.info.height = occupancy_grid->height_;
//        EigIsometry3d occupancy_grid_origin = occupancy_grid->initial_pose_;
//        EigQuaterniond q(occupancy_grid_origin.linear());
//        occupancy_grid_msg.info.origin.orientation.x = q.x();
//        occupancy_grid_msg.info.origin.orientation.y = q.y();
//        occupancy_grid_msg.info.origin.orientation.z = q.z();
//        occupancy_grid_msg.info.origin.orientation.w = q.w();
//        occupancy_grid_msg.info.origin.position.x = occupancy_grid_origin.translation().x();
//        occupancy_grid_msg.info.origin.position.y = occupancy_grid_origin.translation().y();
//        occupancy_grid_msg.info.origin.position.z = occupancy_grid_origin.translation().z();
//        for (int &i: occupancy_grid->data_) {
//            occupancy_grid_msg.data.emplace_back(i);
//        }
//        occgrid_map_pub->publish(occupancy_grid_msg);
//
//        // marker array initialized
//        visualization_msgs::msg::MarkerArray markers;
//        markers.markers.resize(1);
//
//        // node markers visualization
//        visualization_msgs::msg::Marker &traj_marker = markers.markers[0];
//        traj_marker.header.frame_id = global_frame;
//        traj_marker.header.stamp = node->now();
//        traj_marker.ns = "nodes";
//        traj_marker.id = 0;
//        traj_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
//        traj_marker.pose.orientation.w = 1.0;
//        traj_marker.scale.x = traj_marker.scale.y = traj_marker.scale.z = 0.5;
//        traj_marker.points.resize(poses.size());
//        traj_marker.colors.resize(poses.size());
//
//        for (int i = 0; i < poses.size(); i++) {
//            Eigen::Vector3d pos = poses[i].translation();
//            traj_marker.points[i].x = pos.x();
//            traj_marker.points[i].y = pos.y();
//            traj_marker.points[i].z = pos.z();
//
//            double p = static_cast<double>(i) / poses.size();
//            traj_marker.colors[i].r = 1.0 - p;
//            traj_marker.colors[i].g = p;
//            traj_marker.colors[i].b = 0.0;
//            traj_marker.colors[i].a = 1.0;
//        }
//
//        markers_pub->publish(markers);
//
//        // pose message
//        geometry_msgs::msg::PoseStamped pose_;
//        pose_.header.frame_id = global_frame;
//        pose_.header.stamp = stamp;
//        tf2::Quaternion q_;
//        tf2::Matrix3x3(poses[poses.size() - 1].rotation().coeff(0, 0), poses[poses.size() - 1].rotation().coeff(0, 1),
//                       poses[poses.size() - 1].rotation().coeff(0, 2), poses[poses.size() - 1].rotation().coeff(1, 0),
//                       poses[poses.size() - 1].rotation().coeff(1, 1), poses[poses.size() - 1].rotation().coeff(1, 2),
//                       poses[poses.size() - 1].rotation().coeff(2, 0), poses[poses.size() - 1].rotation().coeff(2, 1),
//                       poses[poses.size() - 1].rotation().coeff(2, 2)).getRotation(q_);
//
//        pose_.pose.position.x = poses[poses.size() - 1].translation().x();
//        pose_.pose.position.y = poses[poses.size() - 1].translation().y();
//        pose_.pose.position.z = poses[poses.size() - 1].translation().z();
//        tf2::convert(q_, pose_.pose.orientation);
//        pose_pub->publish(pose_);
//
//        try {
//            tf2::Duration duration(std::chrono::nanoseconds(stamp.nanoseconds()));
//            tf2::TimePoint timepoint(duration);
//
//            tf2::Stamped<tf2::Transform> base_to_map(
//                    tf2::Transform(q_, tf2::Vector3(pose_.pose.position.x, pose_.pose.position.y, 0.0)).inverse(),
//                    //stamp, base_frame);
//                    timepoint, base_frame);
//
//            geometry_msgs::msg::TransformStamped base_to_map_msg, odom_to_map_msg;
//            tf2::convert(base_to_map, base_to_map_msg);
//            odom_to_map_msg = tf_buffer->transform(base_to_map_msg, odom_frame);
//            tf2::Stamped<tf2::Transform> odom_to_map;
//            tf2::convert(odom_to_map_msg, odom_to_map);
//
//            tf2::convert(odom_to_map.inverse(), latest_transform.transform);
//            latest_transform.header.frame_id = global_frame;
//            latest_transform.child_frame_id = odom_frame;
//        } catch (tf2::TransformException) {
//            std::cerr << "Missing tf!" << std::endl;
//        }
//    }
//
//    void BridgeVisualizer::update_odometry_observer(const OdometryStamped3D_MSG::ConstPtr &message) {
//        nav_msgs::msg::Odometry odom_msg;
//
//        rclcpp::Time stamp(message->header_.timestamp_ / 1000000000ll, message->header_.timestamp_ % 1000000000ll);
//
//        odom_msg.header.stamp = stamp;
//        odom_msg.header.frame_id = global_frame;
//
//        EigQuaterniond q(message->odometry_.block<3, 3>(0, 0).cast<double>());
//        odom_msg.pose.pose.orientation.x = q.x();
//        odom_msg.pose.pose.orientation.y = q.y();
//        odom_msg.pose.pose.orientation.z = q.z();
//        odom_msg.pose.pose.orientation.w = q.w();
//
//        odom_msg.pose.pose.position.x = message->odometry_(0, 3);
//        odom_msg.pose.pose.position.y = message->odometry_(1, 3);
//        odom_msg.pose.pose.position.z = message->odometry_(2, 3);
//
//        odom_pub->publish(odom_msg);
//    }
} // namespace lots::slam::wrapper
