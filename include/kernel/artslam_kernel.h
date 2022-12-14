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
                /* front-end */
                // TODO: make a vector
                ARTSLAMLidar lidar;
                ARTSLAMImu imu;
                ARTSLAMGnss gnss;

                /* loop-detectors */
                // TODO: make a vector
                ARTSLAMLoopDetector loop_detectors;

                /* back-end */
                ARTSLAMBackEnd backend;

                ARTSLAMKernel(){};

                void start(ros::NodeHandle* mt_nh, ARTSLAMBridgeVisualizer* bridge, std::string config_file);
        };
    }
}

#endif // ARTSLAM_KERNEL_H
