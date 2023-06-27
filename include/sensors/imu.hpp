// -*- lots-wrapper-c++ -*-

/* ---------------------------------------------------------------------------------------------------------------------
 * Package: LOTS-WRAPPER
 * Class: Imu
 * Author: Mirko Usuelli
 * Advisor: Prof. Matteo Matteucci, PhD
 * Co-Advisors: Matteo Frosi, PhD; Gianluca Bardaro, PhD; Simone Mentasti, PhD; Paolo Cudrano, PhD Student.
 * University: Politecnico di Milano - Artificial Intelligence & Robotics Lab
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
#ifndef WRAPPER_IMU_H
#define WRAPPER_IMU_H

#include "sensor.hpp"

// ROS messages
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>

// TFs libraries
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/transform_listener.h>

using namespace std::placeholders;

namespace lots::slam::wrapper {
    /**
     * Imu
     *
     * ...
     */
    class Imu : public Sensor<sensor_msgs::msg::Imu> {
    public:
        explicit Imu(int id) {
            _sensor_type = IMU;
            _sensor_id = id;

            _start_color = "\033[1;32m";
            _end_color = "\033[0m";

//            _topic = topic;
//            _buffer = buffer;
            _count = 0;
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
//            sensor_sub = n->create_subscription<sensor_msgs::msg::Imu>(_topic, sensor_qos,
//                                                                       std::bind(&lots::slam::wrapper::Imu::callback,
//                                                                                 this, _1));
//        };

        /**
         * Sensor callback for the subscriber interaction.
         *
         * @param msg Template message which depends from the sensor type.
         */
        void callback(const sensor_msgs::msg::Imu &msg) override {
            IMU_MSG::Ptr _imu_msg(new IMU_MSG);

            _imu_msg->header_.timestamp_ =
                    (unsigned long) (msg.header.stamp.sec) * 1000000000ull + (unsigned long) (msg.header.stamp.nanosec);

            _imu_msg->header_.sequence_ = _count;
            _count++;
            // TODO change message content
//            _imu_msg->header_.frame_id_ = msg.header.frame_id;
//            _imu_msg->has_orientation_ = false;
//            _imu_msg->has_linear_acceleration_ = true;
//            _imu_msg->linear_acceleration_ << msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z;
//            _imu_msg->has_angular_velocity_ = true;
//            _imu_msg->angular_velocity_ << msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z;

            backend->backend_handler->update_imu_observer(_imu_msg, "raw");
        };
    private:
        std::string _base_frame;
        int _count;
    };
}

#endif // WRAPPER_IMU_H
