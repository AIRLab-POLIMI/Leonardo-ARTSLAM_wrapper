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

#include "sensor.h"

// ROS messages
#include <sensor_msgs/Imu.h>

// TFs libraries
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>


namespace artslam
{
    namespace lots
    {
        namespace wrapper
        {
            /**
             * Imu
             *
             * ...
             */
            class Imu : public Sensor<sensor_msgs::ImuPtr>
            {
                public:
                    Imu(tf::TransformListener* _tf_listener, int id, std::string topic, int buffer)
                    {
                        _prefilterer = true;
                        _tracker = false;
                        _ground_detector = false;

                        _sensor_type = "IMU";
                        _sensor_id = id;

                        _start_color = "\033[1;32m";
                        _end_color = "\033[0m";

                        _topic = topic;
                        _buffer = buffer;
                        tf_listener = _tf_listener;
                    };

                    void setSubscriber(ros::NodeHandle* mt_nh)
                    {
                        sensor_sub = mt_nh->subscribe(_topic, _buffer, &artslam::lots::wrapper::Imu::callback, this);
                    };

                    /**
                     * Sensor callback for the subscriber interaction.
                     *
                     * @param msg Template message which depends from the sensor type.
                     */
                    void callback(const sensor_msgs::ImuPtr& msg) override
                    {
                        const auto& imu_orientation = msg->orientation;
                        const auto& imu_acceleration = msg->linear_acceleration;

                        geometry_msgs::Vector3Stamped acc_imu;
                        geometry_msgs::Vector3Stamped acc_base;
                        geometry_msgs::QuaternionStamped quat_imu;
                        geometry_msgs::QuaternionStamped quat_base;

                        quat_imu.header.frame_id = acc_imu.header.frame_id = msg->header.frame_id;
                        quat_imu.header.stamp = acc_imu.header.stamp = ros::Time(0);
                        acc_imu.vector = msg->linear_acceleration;
                        quat_imu.quaternion = msg->orientation;

                        try
                        {
                            tf_listener->transformVector("base_link", acc_imu, acc_base);
                            tf_listener->transformQuaternion("base_link", quat_imu, quat_base);
                        }
                        catch (tf2::TransformException &ex)
                        {
                            std::cerr << ex.what() << std::endl;
                            return;
                        }

                        IMU3D_MSG::Ptr conv_imu_msg(new IMU3D_MSG);
                        conv_imu_msg->header_.timestamp_ = msg->header.stamp.toNSec();
                        conv_imu_msg->header_.frame_id_ = "base_link";
                        tf::Stamped<tf::Quaternion> quat;
                        tf::quaternionStampedMsgToTF(quat_base, quat);
                        tf::quaternionTFToEigen(quat, conv_imu_msg->orientation_);
                        tf::Stamped<tf::Vector3> acc;
                        tf::vector3StampedMsgToTF(acc_base, acc);
                        tf::vectorTFToEigen(acc, conv_imu_msg->linear_acceleration_);

                        backend->backend_handler->update_raw_imu_observer(conv_imu_msg);
                    };
            };
        }
    }
}

#endif // WRAPPER_IMU_H
