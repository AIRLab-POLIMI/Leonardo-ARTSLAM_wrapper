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

#include "sensor.h"

// ROS messages
#include <nav_msgs/Odometry.h>

// PCL libraries
#include <tf_conversions/tf_eigen.h>


namespace artslam
{
    namespace lots
    {
        namespace wrapper
        {
            /**
             * Odom
             *
             * ...
             */
            class Odom : public Sensor<nav_msgs::OdometryConstPtr>
            {
                public:
                    Lidar* _lidar;

                    Odom(int id, std::string topic, int buffer)
                    {
                        _prefilterer = false;
                        _tracker = true;
                        _ground_detector = false;

                        _sensor_type = "ODOM";
                        _sensor_id = id;

                        _start_color = "\033[1;34m";
                        _end_color = "\033[0m";

                        _topic = topic;
                        _buffer = buffer;
                    };

                    /**
                     *
                     * @param lidar
                     */
                    void addPriorSensor(Lidar* lidar) {
                        _lidar = lidar;
                    }

                    /**
                     * Method in charge to initialize the sensor ROS subscriber.
                     *
                     * @param mt_nh ROS Node Handler reference
                     */
                    void setSubscriber(ros::NodeHandle* mt_nh)
                    {
                        sensor_sub = mt_nh->subscribe(_topic, _buffer, &artslam::lots::wrapper::Odom::callback, this);
                    };

                    /**
                     * Sensor callback for the subscriber interaction.
                     *
                     * @param msg Template message which depends from the sensor type.
                     */
                    void callback(const nav_msgs::OdometryConstPtr& msg) override
                    {
                        tf::Pose tf_pose;
                        Eigen::Isometry3d i3d;
                        tf::poseMsgToTF(msg->pose.pose, tf_pose);
                        tf::poseTFToEigen(tf_pose, i3d);

                        OdometryStamped3D_MSG::Ptr odom_msg(new OdometryStamped3D_MSG());
                        odom_msg->header_.timestamp_ = msg->header.stamp.toNSec();
                        odom_msg->header_.frame_id_ = "base_link";
                        odom_msg->header_.sensor_type_ = boost::algorithm::to_lower_copy(_sensor_type);;
                        odom_msg->header_.sensor_id_ = _sensor_id;
                        odom_msg->odometry_ = i3d.matrix().cast<float>();
                        odom_msg->covariance_ = Eigen::MatrixXd::Map(&(msg->pose.covariance[0]), 6, 6);

                        // TODO: move to the right front-end
                        (static_cast<LidarTracker*>(_lidar->frontend.modules["tracker"].get()))->update_prior_odometry_observer(odom_msg);
                        //frontend.tracker->update_odometry_observer(odom_msg);
                    };
            };
        }
    }
}

#endif // WRAPPER_ODOM_H
