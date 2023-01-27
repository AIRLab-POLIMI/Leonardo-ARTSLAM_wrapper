// -*- lots-slam-c++ -*-

/* ---------------------------------------------------------------------------------------------------------------------
 * Package: LOTS-WRAPPER
 * Class: Skeleton
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
#ifndef WRAPPER_SKELETON_H
#define WRAPPER_SKELETON_H

// kernel sub-modules
#include "frontend.h"
#include "loop_detector.h"
#include "backend.h"


// sensors
#include "../sensors/lidar.h"
#include "../sensors/imu.h"
#include "../sensors/gnss.h"
#include "../sensors/camera.h"
#include "../sensors/radar.h"
#include "../sensors/odom.h"

#include <tf/transform_listener.h>

namespace artslam
{
    namespace lots
    {
        namespace wrapper
        {
            /**
             * Skeleton
             *
             * This class incorporates all LOTS-SLAM sub-modules (Back-End + Loop Detector + Front-End) and it is in charge
             * of construct them based on the configuration file.
             */
            class Skeleton
            {
                public:
                    tf::TransformListener tf_listener;

                    /* front-end */
                    std::vector<Lidar> lidar_list;
                    std::vector<Imu> imu_list;
                    std::vector<Gnss> gnss_list;
                    std::vector<Camera> camera_list;
                    std::vector<Radar> radar_list;
                    std::vector<Odom> odom_list;

                    /* loop-detectors */
                    std::vector<LoopDetector> loop_detector_list;

                    /* back-end */
                    Backend backend;

                    Skeleton(){};

                    void start(ros::NodeHandle* mt_nh, BridgeVisualizer* bridge, std::string config_file);

                    void addLoopDetector(int id, std::string name) { loop_detector_list.push_back(LoopDetector(id, name)); };

                    void addLidar(int id, std::string topic, int buffer) { lidar_list.push_back(Lidar(id, topic, buffer)); };

                    void addImu(int id, std::string topic, int buffer) { imu_list.push_back(Imu(&tf_listener, id, topic, buffer)); };

                    void addGnss(int id, std::string topic, int buffer) { gnss_list.push_back(Gnss(&tf_listener, id, topic, buffer)); };

                    void addCamera(int id, std::string topic, int buffer) { camera_list.push_back(Camera(id, topic, buffer)); };

                    void addRadar(int id, std::string topic, int buffer) { radar_list.push_back(Radar(id, topic, buffer)); };

                    void addOdom(int id, std::string topic, int buffer) { odom_list.push_back(Odom(id, topic, buffer)); };
            };
        }
    }
}

#endif // WRAPPER_SKELETON_H
