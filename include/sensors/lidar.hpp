// -*- lots-wrapper-c++ -*-

/* ---------------------------------------------------------------------------------------------------------------------
 * Package: LOTS-WRAPPER
 * Class: Lidar
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
#ifndef WRAPPER_LIDAR_H
#define WRAPPER_LIDAR_H

#include "sensor.hpp"

// ROS messages
#include <sensor_msgs/msg/point_cloud2.hpp>
//#include <nav_msgs/msg/odometry.hpp>

// PCL libraries
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2/LinearMath/Matrix3x3.h>
//#include <tf2_eigen/tf2_eigen.hpp>

using namespace std::placeholders;

namespace lots::slam::wrapper {
    using namespace lots::slam::lidar;
    /**
     * Lidar
     *
     * ...
     */
    class Lidar : public Sensor<sensor_msgs::msg::PointCloud2> {
    public:
        explicit Lidar(int id) {
            _sensor_type = LIDAR;
            _sensor_id = id;

            _start_color = "\033[1;33m";
            _end_color = "\033[0m";

//            _topic = topic;
//            _buffer = buffer;
        };

        /**
         * Method in charge to initialize the sensor ROS subscriber.
         *
         * @param n ROS Node reference
         */
//        void setSubscribers(std::shared_ptr<rclcpp::Node> n) override {
//            n->get_parameter<std::string>("base_frame", _base_frame);
//
//            auto sensor_qos = rclcpp::QoS(rclcpp::SensorDataQoS());
//            sensor_sub = n->create_subscription<sensor_msgs::msg::PointCloud2>(_topic, sensor_qos, std::bind(
//                    &lots::slam::wrapper::Lidar::callback, this, _1));
//
//            if (_prior_odom_topic != "/0") {
//                prior_odom_sub = n->create_subscription<nav_msgs::msg::Odometry>(_prior_odom_topic, _buffer, std::bind(
//                        &lots::slam::wrapper::Lidar::prior_odom_callback, this, _1));
//            }
//        };

//        void setBridge(BridgeVisualizer *bridge) {
//            (static_cast<LiDARTracker *>(frontend.modules["tracker"].get()))->register_slam_output_observer(bridge);
//        }

//        void enableIMUCoupling(std::shared_ptr<rclcpp::Node> n, const std::string& topic_name) {
//            auto sensor_qos = rclcpp::QoS(rclcpp::SensorDataQoS());
//            imu_sub = n->create_subscription<sensor_msgs::msg::Imu>(topic_name, sensor_qos, std::bind(
//                    &lots::slam::wrapper::Lidar::imu_callback, this, _1));
//            std::cout<<"IMU tight coupling enabled"<<std::endl;
//        }

        /**
         * Sensor callback for the subscriber interaction.
         *
         * @param msg Template message which depends from the sensor type.
         */
        void callback(const sensor_msgs::msg::PointCloud2 &msg) override {
            pcl::PointCloud<Point3I>::Ptr cloud(new pcl::PointCloud<Point3I>());
            pcl::fromROSMsg(msg, *cloud);
            cloud->header.seq = counter;

            cloud->header.stamp =
                    (unsigned long) (msg.header.stamp.sec) * 1000000000ull + (unsigned long) (msg.header.stamp.nanosec);

            (static_cast<LiDARPrefilterer *>(frontend.modules["prefilterer"].get()))->update_point_cloud_observer(cloud, "raw");
            counter++;
        };

        void imu_callback(const sensor_msgs::msg::Imu &msg) {
            IMU_MSG::Ptr _imu_msg(new IMU_MSG);

            _imu_msg->header_.timestamp_ = msg.header.stamp.sec * 1000000000ull + msg.header.stamp.nanosec;
            _imu_msg->header_.frame_id_ = msg.header.frame_id;
            _imu_msg->linear_acceleration_.value_ << msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z;
            _imu_msg->linear_acceleration_.covariance_ = EigMatrix3d::Identity();
            _imu_msg->angular_velocity_.value_ << msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z;
            _imu_msg->angular_velocity_.covariance_ = EigMatrix3d::Identity();

            (static_cast<LiDARTracker*>(frontend.modules["tracker"].get()))->update_imu_observer(_imu_msg, "raw");
        }

        /**
         * Sensor callback for the prior odom topic stram.
         *
         * @param msg Odometry message.
         */
        void prior_odom_callback(const nav_msgs::msg::Odometry &msg) override {
            //TODO fix this
            return;
            Odometry_MSG::Ptr odom_msg(new Odometry_MSG());

            odom_msg->header_.timestamp_ =
                    (unsigned long) (msg.header.stamp.sec) * 1000000000ull + (unsigned long) (msg.header.stamp.nanosec);

            odom_msg->header_.frame_id_ = msg.header.frame_id;
            // TODO change message to enum
//            odom_msg->header_.sensor_type_ = boost::algorithm::to_lower_copy(_sensor_type);
            odom_msg->header_.sensor_id_ = _sensor_id;
            tf2::Quaternion quat;
            tf2::convert(msg.pose.pose.orientation, quat);
            tf2::Matrix3x3 mat(quat);
            // TODO
//            odom_msg->odometry_ << mat[0][0], mat[0][1], mat[0][2], msg.pose.pose.position.x,
//                    mat[1][0], mat[1][1], mat[1][2], msg.pose.pose.position.y,
//                    mat[2][0], mat[2][1], mat[2][2], msg.pose.pose.position.z,
//                    0.0, 0.0, 0.0, 1.0;

            odom_msg->covariance_ = Eigen::MatrixXd::Map(&(msg.pose.covariance[0]), 6, 6);

            (static_cast<LiDARTracker *>(frontend.modules["tracker"].get()))->update_odometry_observer(odom_msg, "raw");
        };
    public:
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
    };
}

#endif // WRAPPER_LIDAR_H
