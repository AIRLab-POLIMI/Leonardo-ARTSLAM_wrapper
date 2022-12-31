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
#include <artslam_io/pointcloud_io.h>

namespace artslam
{
    namespace lots
    {
        namespace wrapper
        {
            /**
             * FrontEnd
             *
             * For each sensor a front-end module is initialized because each sub-module MUST be sensor dependent and cannot
             * neither be shared among sensor of the same type; thus it is used by only one sensor (where it is called).
             */
            class Frontend
            {
                public:
                    std::shared_ptr<LidarPrefilterer> prefilterer;
                    std::shared_ptr<LidarTracker> tracker;
                    std::shared_ptr<LidarGroundDetector> ground_detector;

                    Frontend(){};

                    void start(std::string config_file, bool _prefilterer, bool _tracker, bool _ground_detector);
            };
        }
    }
}

#endif // WRAPPER_FRONTEND_H
