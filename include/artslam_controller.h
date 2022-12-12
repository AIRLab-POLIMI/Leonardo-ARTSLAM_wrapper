// -*- artslam-c++ -*-

/* ---------------------------------------------------------------------------------------------------------------------
 * Package: ARTSLAM-WRAPPER
 * Class: ARTSLAMController
 * Author: Mirko Usuelli
 * Advisor: Prof. Matteo Matteucci, PhD
 * Co-Advisors: Matteo Frosi, PhD; Gianluca Bardaro, PhD; Simone Mentasti, PhD; Paolo Cudrano, PhD Student.
 * University: Politecnico di Milano - Artificial Intelligence & Robotics Lab
 * Last Modified: 30/11/2022
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
#ifndef ARTSLAM_CONTROLLER_H
#define ARTSLAM_CONTROLLER_H

// ARTSLAMBridgeVisualizer
#include "artslam_bridge_visualizer.h"
#include "artslam_kernel.h"
#include "sensors/artslam_lidar.h"
#include "sensors/artslam_imu.h"
#include "sensors/artslam_gnss.h"

// ROS libraries
#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>

// ROS messages
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64.h>

// ROS service
#include <artslam_wrapper/OfflineSLAM.h>  // <------------------------------- // TODO: change name

// PCL libraries
#include <pcl_conversions/pcl_conversions.h>

// TFs libraries
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>

// Nodelet libraries
#include <nodelet/nodelet.h>


namespace artslam
{
    namespace laser3d
    {
        /**
         * ARTSLAMController
         *
         * ...
         */
        class ARTSLAMController : public nodelet::Nodelet
        {
            public:
                /* Attributes --------------------------------------------------------------------------------------- */

                /* Methods ------------------------------------------------------------------------------------------ */
                // Constructor
                ARTSLAMController();
                void onInit();
                void startKernel(std::string config_file, std::string results_path);
                bool offline_slam(artslam_wrapper::OfflineSLAM::Request &req,
                                  artslam_wrapper::OfflineSLAM::Request &res);

            private:
                void get_filepaths(const std::string &path, const std::string &extension, std::vector<std::string> &filepaths);

                /* Attributes --------------------------------------------------------------------------------------- */
                ros::NodeHandle mt_nh;
                ros::NodeHandle private_nh;
                ros::ServiceServer service;

                // general params
                double param_value;
                std::string config_file;
                std::string results_path;

                // ART-SLAM wrapper components
                ARTSLAMBridgeVisualizer bridge;
                ARTSLAMKernel kernel;

                // Sensors
                ARTSLAMLidar lidar;
                ARTSLAMImu imu;
                ARTSLAMGnss gnss;
                // std::list<ARTSLAMSensor*> sensors;

                /* Methods ------------------------------------------------------------------------------------------ */

        };
    }
}

#endif // ARTSLAM_CONTROLLER_H
