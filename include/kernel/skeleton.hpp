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
#include "backend.hpp"
#include "frontend.hpp"
#include "loop_detector.hpp"

// sensors
#include "../sensors/camera.hpp"
#include "../sensors/gnss.hpp"
#include "../sensors/imu.hpp"
#include "../sensors/lidar.hpp"
#include "../sensors/odom.hpp"
#include "../sensors/radar.hpp"

//#include <tf2_ros/transform_listener.h>

namespace lots::slam::wrapper {
    /**
     * Skeleton
     *
     * This class incorporates all LOTS-SLAM sub-modules (Back-End + Loop Detector + Front-End) and it is in charge
     * of construct them based on the configuration file.
     */
    class Skeleton {
    public:
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

        Skeleton() = default;

//        void start(std::shared_ptr<rclcpp::Node> n, BridgeVisualizer *bridge, const std::string& config_file);
        void start(std::shared_ptr<rclcpp::Node> n, const std::string& config_file);
        void registerObserver(SLAMOutputObserver *observer);

        void addLoopDetector(int id) { loop_detector_list.emplace_back(id); };
    };
}

#endif // WRAPPER_SKELETON_H
