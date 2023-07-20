// -*- lots-wrapper-c++ -*-

/* ---------------------------------------------------------------------------------------------------------------------
 * Package: LOTS-WRAPPER
 * Class: Frontend
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
#ifndef WRAPPER_FRONTEND_H
#define WRAPPER_FRONTEND_H

// ARTSLAM libraries
#include <utils/configuration_parser.h>
#include <frontend/lidar/lidar_registration.h>
#include <frontend/lidar/lidar_tracker.h>
#include <frontend/lidar/lidar_ground_detector.h>
#include <frontend/lidar/lidar_prefilterer.h>
//#include <frontend/odom/odom_tracker.h>
//#include <artslam_io/pointcloud_io.h>
#include <map>
#include <string>
//#include <controller/bridge_visualizer.hpp>

namespace lots::slam::wrapper {
    using namespace lots::core::types;
    /**
     * FrontEnd
     *
     * For each sensor a front-end module is initialized because each sub-module MUST be sensor dependent and cannot
     * neither be shared among sensor of the same type; thus it is used by only one sensor (where it is called).
     */
    class Frontend {
    public:
        std::map<std::string, std::shared_ptr<FrontEndModule>> modules;

        Frontend() = default;

        void start(std::string config_file, SensorType sensor, int id);

        void addPrefilterer(std::string config_file, SensorType sensor, int id);

        void addTracker(std::string config_file, SensorType sensor, int id);

        void addGroundDetector(std::string config_file, SensorType sensor, int id);
    };
}

#endif // WRAPPER_FRONTEND_H
