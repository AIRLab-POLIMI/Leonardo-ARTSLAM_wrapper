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
#include <frontend/frontend_module.h>
#include <sensors/lidar/lidar_registration.h>
#include <sensors/lidar/lidar_tracker.h>
#include <sensors/lidar/lidar_ground_detector.h>
#include <sensors/lidar/lidar_prefilterer.h>
#include <sensors/odom/odom_tracker.h>
#include <artslam_io/pointcloud_io.h>
#include <map>
#include <string>
#include <controller/bridge_visualizer.hpp>

namespace artslam::lots::wrapper {
    /**
     * FrontEnd
     *
     * For each sensor a front-end module is initialized because each sub-module MUST be sensor dependent and cannot
     * neither be shared among sensor of the same type; thus it is used by only one sensor (where it is called).
     */
    class Frontend {
    public:
        std::map<std::string, std::shared_ptr<FrontendModule>> modules;

        Frontend() = default;

        void start(std::string config_file, std::string sensor, int id);

        void addPrefilterer(std::string config_file, std::string sensor, int id);

        void addTracker(std::string config_file, std::string sensor, int id);

        void addGroundDetector(std::string config_file, std::string sensor, int id);
    };
}

#endif // WRAPPER_FRONTEND_H
