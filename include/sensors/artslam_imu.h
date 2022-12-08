// -*- artslam-c++ -*-

/* ---------------------------------------------------------------------------------------------------------------------
 * Package: ARTSLAM-WRAPPER
 * Class: ARTSLAMController
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
#ifndef ARTSLAM_IMU_H
#define ARTSLAM_IMU_H

// ROS messages
#include <sensor_msgs/Imu.h>

// TFs libraries
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>


namespace artslam
{
    namespace laser3d
    {
        /**
         * ARTSLAMImu
         *
         * ...
         */
        class ARTSLAMImu
        {
            private:
                tf::TransformListener tf_listener;

            public:
                /* Attributes --------------------------------------------------------------------------------------- */
                int counter = 0;
                ros::Subscriber sub;
                ARTSLAMKernel* kernel;

                /* Methods ------------------------------------------------------------------------------------------ */
                ARTSLAMImu(){};

                void setKernel(ARTSLAMKernel* artslam_kernel)
                {
                    kernel = artslam_kernel;
                };

                void callback(const sensor_msgs::ImuPtr& msg)
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
                        tf_listener.transformVector("base_link", acc_imu, acc_base);
                        tf_listener.transformQuaternion("base_link", quat_imu, quat_base);
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

                    kernel->backend_handler->update_raw_imu_observer(conv_imu_msg);
                };
        };
    }
}

#endif // ARTSLAM_IMU_H
