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
#include "kernel/artslam_kernel.h"

namespace artslam
{
    namespace laser3d
    {
        /**
         * Initialize kernel modules (Back-End + Loop Detector + Front-End).
         *
         * @param mt_nh ROS node handler
         * @param bridge BridgeVisualizer reference
         * @param config_file Configuration file
         */
        void ARTSLAMKernel::start(ros::NodeHandle* mt_nh, ARTSLAMBridgeVisualizer* bridge, std::string config_file)
        {
            // back-end initialization
            backend.start(bridge, config_file);

            // loop detectors initialization
            loop_detectors.start(config_file);

            // front-end initialization
            lidar.start(mt_nh, &backend, config_file);
            imu.start(mt_nh, &backend, config_file);
            gnss.start(mt_nh, &backend, config_file);

            // linking back-end with loop detectors
            backend.backend_handler->set_loop_detector(loop_detectors.loop_detector.get());
        }
    }
}
