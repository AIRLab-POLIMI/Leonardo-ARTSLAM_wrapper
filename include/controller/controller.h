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
#include "bridge_visualizer.h"
#include "../kernel/skeleton.h"

#include "../kernel/frontend.h"
#include "../sensors/lidar.h"
#include "../sensors/imu.h"
#include "../sensors/gnss.h"

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
#include <artslam_wrapper/OfflineSLAM.h>

// PCL libraries
#include <pcl_conversions/pcl_conversions.h>

// TFs libraries
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>

// Nodelet libraries
#include <nodelet/nodelet.h>
#include <utils/configuration_parser.h>


namespace artslam::lots::wrapper
{
            /**
             * Controller
             *
             * Main wrapper module which triggers the whole process: it reads a configuration file and configures the
             * ARTSLAM kernel with the desired sensors by allocating front-ends and also the desired loop detectors.
             * The controller is also in charge to manage the bridge-visualizer.
             */
            class Controller
            {
                private:
                    /* Attributes ----------------------------------------------------------------------------------- */
                    // ros attributes
                    ros::NodeHandle mt_nh;
                    ros::NodeHandle private_nh;
                    ros::ServiceServer service;

                    // general params
                    double param_value;
                    std::string config_file;
                    std::string results_path;

                    // wrapper main components
                    BridgeVisualizer bridge;
                    Skeleton skeleton;
            public:
                    /* Methods -------------------------------------------------------------------------------------- */
                    Controller();
                    bool offline_slam(artslam_wrapper::OfflineSLAM::Request &req,
                                      artslam_wrapper::OfflineSLAM::Response &res);
            };
}

#endif // WRAPPER_CONTROLLER_H
