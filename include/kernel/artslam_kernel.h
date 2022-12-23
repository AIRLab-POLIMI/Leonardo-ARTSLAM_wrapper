// -*- artslam-c++ -*-

/* ---------------------------------------------------------------------------------------------------------------------
 * Package: ARTSLAM-WRAPPER
 * Class: ARTSLAMKernel
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
#ifndef ARTSLAM_KERNEL_H
#define ARTSLAM_KERNEL_H

// kernel sub-modules
#include "artslam_front_end.h"
#include "artslam_loop_detector.h"
#include "artslam_back_end.h"

// sensors
#include "sensors/artslam_lidar.h"
#include "sensors/artslam_imu.h"
#include "sensors/artslam_gnss.h"
#include "sensors/artslam_camera.h"
#include "sensors/artslam_radar.h"
#include "sensors/artslam_odom.h"

#include <tf/transform_listener.h>

namespace artslam
{
    namespace laser3d
    {
        /**
         * ARTSLAMKernel
         *
         * This class incorporates all ARTSLAM sub-modules (Back-End + Loop Detector + Front-End) and it is in charge
         * of construct them based on the configuration file.
         */
        class ARTSLAMKernel
        {
            public:
                tf::TransformListener tf_listener;

                /* front-end */
                std::vector<ARTSLAMLidar> lidar_list;
                std::vector<ARTSLAMImu> imu_list;
                std::vector<ARTSLAMGnss> gnss_list;
                std::vector<ARTSLAMCamera> camera_list;
                std::vector<ARTSLAMRadar> radar_list;
                std::vector<ARTSLAMOdom> odom_list;

                /* loop-detectors */
                std::vector<ARTSLAMLoopDetector> loop_detector_list;

                /* back-end */
                ARTSLAMBackEnd backend;

                ARTSLAMKernel(){};

                void start(ros::NodeHandle* mt_nh, ARTSLAMBridgeVisualizer* bridge, std::string config_file);

                void addLoopDetector(int id) { loop_detector_list.push_back(ARTSLAMLoopDetector(id)); };

                void addLidar(int id, std::string topic, int buffer) { lidar_list.push_back(ARTSLAMLidar(id, topic, buffer)); };

                void addImu(int id, std::string topic, int buffer) { imu_list.push_back(ARTSLAMImu(&tf_listener, id, topic, buffer)); };

                void addGnss(int id, std::string topic, int buffer) { gnss_list.push_back(ARTSLAMGnss(&tf_listener, id, topic, buffer)); };

                void addCamera(int id, std::string topic, int buffer) { camera_list.push_back(ARTSLAMCamera(id, topic, buffer)); };

                void addRadar(int id, std::string topic, int buffer) { radar_list.push_back(ARTSLAMRadar(id, topic, buffer)); };

                void addOdom(int id, std::string topic, int buffer) { odom_list.push_back(ARTSLAMOdom(id, topic, buffer)); };
        };
    }
}

#endif // ARTSLAM_KERNEL_H
