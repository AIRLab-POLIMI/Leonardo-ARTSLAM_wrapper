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

#include <iostream>
#include <filesystem>
#include <algorithm>
#include <artslam_io/pointcloud_io.h>
#include <artslam_io/kitti_reader.hpp>

using namespace artslam::core::io;

namespace artslam::lots::wrapper
{
    void get_filepaths(const std::string& path, const std::string& extension, std::vector<std::string>& filepaths) {
        for(const auto& p : std::filesystem::directory_iterator(path)) {
            if(p.is_regular_file()) {
                if(p.path().extension().string() == extension) {
                    filepaths.emplace_back(p.path());
                }
            }
        }

        std::sort(filepaths.begin(), filepaths.end());
    }

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

        // ---------------------------
//        std::string config_file = "/home/mirko/Desktop/Code/Thesis/easymile/src/ARTSLAM/configs/KITTI_new.json";
//        std::string save_path = "/home/mirko/Desktop/Code/Thesis/easymile/src/ARTSLAM/results/";
//        std::vector<std::string> slam_paths = parse_slam_paths(config_file);
//
//        // get the filepaths corresponding to the pointclouds
//        std::vector<std::string> pointclouds_filepaths;
//        get_filepaths(slam_paths[0], ".bin", pointclouds_filepaths);
//
//        // get the filepaths corresponding to IMU and GPS data (KITTI)
//        std::vector<std::string> oxts_filepaths;
//        get_filepaths(slam_paths[2], ".txt", oxts_filepaths);
//
//        // read the timestamps for both pointclouds, IMU and GPS data (KITTI)
//        KITTI_Reader kitti_reader({false, boost::log::trivial::trace});
//        std::vector<uint64_t> timestamps, oxts_timestamps;
//        kitti_reader.read_timestamps(slam_paths[1], timestamps);
//        kitti_reader.read_timestamps(slam_paths[3], oxts_timestamps);
//
//        // perform SLAM, playing back
//        for(int i = 0; i < pointclouds_filepaths.size(); i++) {
//            pcl::PointCloud<Point3I>::Ptr pointcloud = kitti_reader.read_pointcloud(pointclouds_filepaths[i]);
//            pointcloud->header.frame_id = "";
//            pointcloud->header.seq = i;
//            pointcloud->header.stamp = timestamps[i];
//
//            (static_cast<LidarPrefilterer*>(skeleton.lidar_list[0].frontend.modules["prefilterer"].get()))->update_raw_pointcloud_observer(pointcloud);
//            usleep(100000); // just to simulate data acquisition rate
//        }
//
//        sleep(5);
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
