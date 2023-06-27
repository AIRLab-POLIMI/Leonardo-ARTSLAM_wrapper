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
#include "controller/controller.hpp"
#include <filesystem>
#include <execution>

using namespace std::placeholders;

namespace lots::slam::wrapper {
// Initialize the node by reading the configuration file and setting the kernel.
Controller::Controller() : param_value(0), bridge(node)
{
    node->declare_parameter("value", rclcpp::ParameterValue(0.0));
    node->declare_parameter("configuration_file", rclcpp::ParameterValue(""));
    node->declare_parameter("results_path", rclcpp::ParameterValue(""));
    node->declare_parameter("base_frame", rclcpp::ParameterValue("base_link"));
    node->declare_parameter("global_frame", rclcpp::ParameterValue("map"));
    node->declare_parameter("odom_frame", rclcpp::ParameterValue("odom"));
    node->declare_parameter("delay", rclcpp::ParameterValue(200000000));

    node->get_parameter<double>("value", param_value);
    node->get_parameter<std::string>("configuration_file", config_file);
    node->get_parameter<std::string>("results_path", results_path);
    node->get_parameter<std::string>("base_frame", bridge.base_frame);
    node->get_parameter<std::string>("global_frame", bridge.global_frame);
    node->get_parameter<std::string>("odom_frame", bridge.odom_frame);
    node->get_parameter<uint>("delay", bridge.delay);

    bridge.init_tf();

    service = node->create_service<std_srvs::srv::Empty>("offline_slam", std::bind(&Controller::offline_slam, this, _1, _2));

    skeleton.start(node, &bridge, config_file);
}

/**
 * ROS Service for the final PCL map.
 *
 * @param req Request
 * @param res Response
 * @return true
 */
bool Controller::offline_slam(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
        std::shared_ptr<std_srvs::srv::Empty::Response> res)
{
     skeleton.backend.backend_handler->save_results(results_path);
     return true;
}

}
