// -*- lots-wrapper-c++ -*-

/* ---------------------------------------------------------------------------------------------------------------------
 * Package: LOTS-WRAPPER
 * Class: Odom
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
#ifndef WRAPPER_ODOM_H
#define WRAPPER_ODOM_H

#include "sensor.hpp"

// ROS messages
#include <nav_msgs/msg/odometry.hpp>

// PCL libraries
//#include <tf2_eigen/tf2_eigen.hpp>

using namespace std::placeholders;


namespace lots::slam::wrapper {
    /**
     * Odom
     *
     * ...
     */
    class Odom : public Sensor<nav_msgs::msg::Odometry> {
    public:
        explicit Odom(int id) {
            _sensor_type = ODOMETRY;
            _sensor_id = id;

            _start_color = "\033[1;34m";
            _end_color = "\033[0m";

//            _topic = topic;
//            _buffer = buffer;
        };

        /**
         * Method in charge to initialize the sensor ROS subscriber.
         *
         * @param n ROS Node reference
         */
//        void setSubscribers(std::shared_ptr<rclcpp::Node> n) {
//            sensor_sub = n->create_subscription<nav_msgs::msg::Odometry>(_topic, _buffer, std::bind(
//                    &lots::slam::wrapper::Odom::callback, this, _1));
//        };

        /**
         * Sensor callback for the subscriber interaction.
         *
         * @param msg Template message which depends from the sensor type.
         */
        void callback(const nav_msgs::msg::Odometry &msg) override {
            // commented out due to the return above (look at lidar.hpp if translation needed)
            /*
            tf2::Pose tf_pose;
            Eigen::Isometry3d i3d;
            tf2::poseMsgToTF(msg->pose.pose, tf_pose);
            tf2::poseTFToEigen(tf_pose, i3d);

            OdometryStamped3D_MSG::Ptr odom_msg(new OdometryStamped3D_MSG());
            odom_msg->header_.timestamp_ = msg.header.stamp.toNSec();
            odom_msg->header_.frame_id_ = "base_link";
            odom_msg->header_.sensor_type_ = boost::algorithm::to_lower_copy(_sensor_type);;
            odom_msg->header_.sensor_id_ = _sensor_id;
            odom_msg->odometry_ = i3d.matrix().cast<float>();
            odom_msg->covariance_ = Eigen::MatrixXd::Map(&(msg.pose.covariance[0]), 6, 6);

            (static_cast<OdomTracker*>(frontend.modules["tracker"].get()))->update_odometry_observer(odom_msg);
            */
        };
    };
}

#endif // WRAPPER_ODOM_H
