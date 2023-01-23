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
#include "controller/controller.h"

namespace artslam::lots::wrapper
{
    // Initialize the node by reading the configuration file and setting the kernel.
    Controller::Controller() : param_value(0), private_nh("~"), mt_nh()
    {
        private_nh.getParam("value", param_value);
        private_nh.getParam("configuration_file", config_file);
        private_nh.getParam("results_path", results_path);
        private_nh.param<std::string>("base_frame", bridge.base_frame, "base_link");
        private_nh.param<std::string>("global_frame", bridge.global_frame, "map");
        private_nh.param<std::string>("odom_frame", bridge.odom_frame, "odom");
        private_nh.param<float>("delay", bridge.delay, 0.2);
        service = private_nh.advertiseService("OfflineSLAM", &Controller::offline_slam, this);

        bridge.set_handler(mt_nh);
        skeleton.start(&mt_nh, &bridge, config_file);
    }

    /**
     * ROS Service for the final PCL map.
     *
     * @param req Request
     * @param res Response
     * @return true
     */
    bool Controller::offline_slam(artslam_wrapper::OfflineSLAM::Request &req, artslam_wrapper::OfflineSLAM::Response &res)
    {
        skeleton.backend.backend_handler->save_results(results_path);
        return true;
    }
}
