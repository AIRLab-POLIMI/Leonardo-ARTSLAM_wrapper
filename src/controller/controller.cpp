// -*- lots-wrapper-c++ -*-

/* ---------------------------------------------------------------------------------------------------------------------
 * Package: LOTS-WRAPPER
 * Class: Controller
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
#include "controller/controller.hpp"
#include <filesystem>
#include <execution>

#include "tf2_sensor_msgs/tf2_sensor_msgs.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>

using namespace std::placeholders;
using namespace std::chrono_literals;

namespace lots::slam::wrapper {
    // Initialize the node by reading the configuration file and setting the kernel.
    Controller::Controller() : param_value(0) {
        node->declare_parameter("value", rclcpp::ParameterValue(0.0));
        node->declare_parameter("configuration_file", rclcpp::ParameterValue(""));
        node->declare_parameter("results_path", rclcpp::ParameterValue(""));
        node->declare_parameter("base_frame", rclcpp::ParameterValue("UGV_base_link"));
        node->declare_parameter("global_frame", rclcpp::ParameterValue("map"));
        node->declare_parameter("odom_frame", rclcpp::ParameterValue("odom"));
        node->declare_parameter("imu_topic", rclcpp::ParameterValue(""));
        node->declare_parameter("delay", rclcpp::ParameterValue(200000000));

        node->get_parameter<double>("value", param_value);
        node->get_parameter<std::string>("configuration_file", config_file);
        node->get_parameter<std::string>("results_path", results_path);
        node->get_parameter<std::string>("base_frame", base_frame);
        node->get_parameter<std::string>("global_frame", global_frame);
        node->get_parameter<std::string>("odom_frame", odom_frame);
        node->get_parameter<uint>("delay", delay);

        tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(node);
        tf_odom_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(node);

        init_tf();

        tf_pub = node->create_wall_timer(200ms, std::bind(&Controller::timer_callback, this));
        service = node->create_service<std_srvs::srv::Empty>("offline_slam",
                                                             std::bind(&Controller::offline_slam, this, _1, _2));
        markers_pub = node->create_publisher<visualization_msgs::msg::MarkerArray>("markers", 10);
        map_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("artslam_map", 1);

        skeleton.start(node, config_file);
        skeleton.registerObserver(this);
        skeleton.registerOdomObserver(this);
    }

    /**
     * ROS Service for the final PCL map.
     *
     * @param req Request
     * @param res Response
     * @return true
     */
    bool Controller::offline_slam(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                                  std::shared_ptr<std_srvs::srv::Empty::Response> res) {
        skeleton.backend.backend_handler->save_results(results_path);
        return true;
    }

    void Controller::init_tf() {
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

    void Controller::timer_callback() {
        if (!latest_transform.header.frame_id.empty()) {
            latest_transform.header.stamp = rclcpp::Time(latest_transform.header.stamp) + (node->now() - last_time);

            tf_broadcaster->sendTransform(latest_transform);
            last_time = node->now();
        }
    }
    
    void Controller::update_slam_output_observer(const SLAMOutput_MSG::Ptr &slam_output, const std::string &id) {
        show_markers(slam_output->poses_.value());
        // show_map(slam_output->map_.value());
        if (slam_output->map_.has_value()) {
            RCLCPP_INFO(node->get_logger(), "HERE IS THE MAP");
            show_map(slam_output->map_.value());
        }

        latest_transform.header.frame_id = global_frame;
        latest_transform.child_frame_id = odom_frame;

        tf2::Quaternion q_;
        tf2::Matrix3x3(slam_output->map_to_odom_->rotation().coeff(0, 0),
                       slam_output->map_to_odom_->rotation().coeff(0, 1),
                       slam_output->map_to_odom_->rotation().coeff(0, 2),
                       slam_output->map_to_odom_->rotation().coeff(1, 0),
                       slam_output->map_to_odom_->rotation().coeff(1, 1),
                       slam_output->map_to_odom_->rotation().coeff(1, 2),
                       slam_output->map_to_odom_->rotation().coeff(2, 0),
                       slam_output->map_to_odom_->rotation().coeff(2, 1),
                       slam_output->map_to_odom_->rotation().coeff(2, 2)).getRotation(q_);

        tf2::convert(q_, latest_transform.transform.rotation);

        latest_transform.transform.translation.x = slam_output->map_to_odom_->translation().x();
        latest_transform.transform.translation.y = slam_output->map_to_odom_->translation().y();
        latest_transform.transform.translation.z = slam_output->map_to_odom_->translation().z();
        auto point_cloud_stamp = slam_output->last_point_cloud_.value()->header.stamp;
        latest_transform.header.stamp = rclcpp::Time(point_cloud_stamp / 1000000000ull,
                                                     point_cloud_stamp % 1000000000ull);

        tf_broadcaster->sendTransform(latest_transform);
        last_time = node->now();
    }

    void Controller::update_slam_output_observer(const SLAMOutput_MSG::ConstPtr &slam_output, const std::string &id) {

    }

    void Controller::update_odometry_observer(Odometry_MSG::Ptr odometry_msg, const std::string& id) {
        odom_transform.header.frame_id = odom_frame;
        odom_transform.child_frame_id = base_frame;

        tf2::Quaternion q_;
        tf2::Matrix3x3(odometry_msg->value_(0, 0),
                       odometry_msg->value_(0, 1),
                       odometry_msg->value_(0, 2),
                       odometry_msg->value_(1, 0),
                       odometry_msg->value_(1, 1),
                       odometry_msg->value_(1, 2),
                       odometry_msg->value_(2, 0),
                       odometry_msg->value_(2, 1),
                       odometry_msg->value_(2, 2)).getRotation(q_);

        tf2::convert(q_, odom_transform.transform.rotation);

        odom_transform.transform.translation.x = odometry_msg->value_(0,3);
        odom_transform.transform.translation.y = odometry_msg->value_(1,3);
        odom_transform.transform.translation.z = odometry_msg->value_(2,3);
        auto point_cloud_stamp = odometry_msg->header_.timestamp_;
        odom_transform.header.stamp = rclcpp::Time(point_cloud_stamp / 1000000000ull,
                                                     point_cloud_stamp % 1000000000ull);

        tf_odom_broadcaster->sendTransform(odom_transform);

    }
    
    void Controller::update_odometry_observer(Odometry_MSG::ConstPtr odometry_msg, const std::string& id) {

    }

    void Controller::show_markers(std::vector<EigIsometry3d> poses) {
        // marker array initialized
        visualization_msgs::msg::MarkerArray markers;
        markers.markers.resize(1);

        // node markers visualization
        visualization_msgs::msg::Marker &traj_marker = markers.markers[0];
        traj_marker.header.frame_id = global_frame;
        traj_marker.header.stamp = node->now();
        traj_marker.ns = "nodes";
        traj_marker.id = 0;
        traj_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        traj_marker.pose.orientation.w = 1.0;
        traj_marker.scale.x = traj_marker.scale.y = traj_marker.scale.z = 0.5;
        traj_marker.points.resize(poses.size());
        traj_marker.colors.resize(poses.size());

        for (int i = 0; i < poses.size(); i++) {
            Eigen::Vector3d pos = poses[i].translation();
            traj_marker.points[i].x = pos.x();
            traj_marker.points[i].y = pos.y();
            traj_marker.points[i].z = pos.z();

            double p = static_cast<double>(i) / poses.size();
            traj_marker.colors[i].r = 1.0 - p;
            traj_marker.colors[i].g = p;
            traj_marker.colors[i].b = 0.0;
            traj_marker.colors[i].a = 1.0;
        }

        markers_pub->publish(markers);
    }

    void Controller::show_map(pcl::PointCloud<Point3I>::ConstPtr map) {
        sensor_msgs::msg::PointCloud2 pointcloud_map;
        //std::make_shared<sensor_msgs::msg::PointCloud2>();

        pcl::toROSMsg(*map, pointcloud_map);
        pointcloud_map.header.frame_id = "map";

        map_pub->publish(pointcloud_map);
    }
}
