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
#ifndef WRAPPER_CONTROLLER_H
#define WRAPPER_CONTROLLER_H

// BridgeVisualizer
#include "../kernel/skeleton.hpp"
//#include "bridge_visualizer.hpp"

//#include "../kernel/frontend.hpp"
//#include "../sensors/gnss.hpp"
//#include "../sensors/imu.hpp"
//#include "../sensors/lidar.hpp"

#include "observers.h"

// ROS libraries
#include "rclcpp/rclcpp.hpp"

// ROS messages
//#include <sensor_msgs/msg/imu.hpp>
//#include <sensor_msgs/msg/nav_sat_fix.hpp>
//#include <sensor_msgs/msg/point_cloud2.hpp>
//#include <std_msgs/msg/float64.hpp>

// ROS service
#include <std_srvs/srv/empty.hpp>
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav_msgs/msg/odometry.hpp"

// PCL libraries
//#include <pcl_conversions/pcl_conversions.h>

// TFs libraries
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
//#include <tf2_ros/transform_listener.h>
//#include <tf2_eigen/tf2_eigen.hpp>

namespace lots::slam::wrapper {
/**
 * Controller
 *
 * Main wrapper module which triggers the whole process: it reads a configuration file and configures the
 * ARTSLAM kernel with the desired sensors by allocating front-ends and also the desired loop detectors.
 * The controller is also in charge to manage the bridge-visualizer.
 */
    class Controller : public SLAMOutputObserver {
    private:
        /* Attributes ----------------------------------------------------------------------------------- */
        std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("offline_slam_server");
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr service;
        rclcpp::TimerBase::SharedPtr tf_pub;
        rclcpp::TimerBase::SharedPtr odom_pub;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr state_pub;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;

        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
        geometry_msgs::msg::TransformStamped latest_transform;
        geometry_msgs::msg::Twist last_speed;
        uint delay;

        // general params
        double param_value;
        std::string config_file;
        std::string results_path;
        std::string base_frame;
        std::string odom_frame;
        std::string global_frame;

        // wrapper main components
//        BridgeVisualizer bridge;
        Skeleton skeleton;

    public:
        /* Methods -------------------------------------------------------------------------------------- */
        Controller();

        bool offline_slam(std::shared_ptr<std_srvs::srv::Empty::Request> req,
                          std::shared_ptr<std_srvs::srv::Empty::Response> res);

        std::shared_ptr<rclcpp::Node> get_node() {
            return node;
        };

        // Observer updating interfaces
        void update_slam_output_observer(const SLAMOutput_MSG::Ptr& slam_output, const std::string& id) override;
        void update_slam_output_observer(const SLAMOutput_MSG::ConstPtr& slam_output, const std::string& id) override;

        void init_tf();
        void timer_callback();
        void odom_timer_callback();
        void odom_callback(nav_msgs::msg::Odometry::SharedPtr msg);
        void show_markers(std::vector<EigIsometry3d> poses);
    };
}

#endif // WRAPPER_CONTROLLER_H
