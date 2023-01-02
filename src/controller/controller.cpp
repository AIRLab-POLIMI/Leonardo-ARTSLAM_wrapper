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

#include <artslam_io/kitti_reader.hpp>
#include <filesystem>
#include <execution>

#include <pluginlib/class_list_macros.h>

namespace artslam
{
    namespace lots
    {
        namespace wrapper
        {
            Controller::Controller() : param_value(0) { return; };

            /**
             * Initialize the node by reading the configuration file and setting the kernel.
             */
            void Controller::onInit()
            {
                private_nh = getPrivateNodeHandle();
                private_nh.getParam("value", param_value);
                private_nh.getParam("configuration_file", config_file);
                private_nh.getParam("results_path", results_path);
                service = private_nh.advertiseService("OfflineSLAM", &Controller::offline_slam, this);

                mt_nh = getMTNodeHandle();
                bridge.set_handler(mt_nh);
                bridge.set_main_reference(parse_main_tf_reference(config_file));
                skeleton.start(&mt_nh, &bridge, config_file);
            }

            /**
             * ROS Service for the final PCL map.
             *
             * @param req Request
             * @param res Response
             * @return true
             */
            bool Controller::offline_slam(artslam_wrapper::OfflineSLAM::Request &req,
                                                 artslam_wrapper::OfflineSLAM::Response &res)
            {
                skeleton.backend.backend_handler->save_results(results_path);
                return true;

                std::vector<std::string> slam_paths = parse_slam_paths(config_file);

                std::vector<std::string> pointclouds_filepaths;
                get_filepaths(slam_paths[0], ".bin", pointclouds_filepaths);

                std::vector<std::string> oxts_filepaths;
                get_filepaths(slam_paths[2], ".txt", oxts_filepaths);

                artslam::core::io::KITTI_Reader kitti_reader({false, boost::log::trivial::trace});
                std::vector<uint64_t> timestamps, oxts_timestamps;
                kitti_reader.read_timestamps(slam_paths[1], timestamps);
                kitti_reader.read_timestamps(slam_paths[3], oxts_timestamps);

                // load immediately all IMU and GNSS data
                for (int i = 0; i < oxts_filepaths.size(); i++) {
                    IMU3D_MSG::Ptr imu3d_msg = kitti_reader.read_imu(oxts_filepaths[i]);
                    imu3d_msg->header_.frame_id_ = "";
                    imu3d_msg->header_.sequence_ = i;
                    imu3d_msg->header_.timestamp_ = oxts_timestamps[i];
                    //prefilterer.update_raw_imu_observer(imu3d_msg);
                    //backend_handler.update_raw_imu_observer(imu3d_msg);

                    GeoPointStamped_MSG::Ptr gnss_msg = kitti_reader.read_gps(oxts_filepaths[i]);
                    gnss_msg->header_.frame_id_ = "";
                    gnss_msg->header_.sequence_ = i;
                    gnss_msg->header_.timestamp_ = oxts_timestamps[i];
                    //backend_handler.update_raw_gnss_observer(gnss_msg);
                }

                for (int i = 0; i < pointclouds_filepaths.size(); i++) {
                    pcl::PointCloud<Point3I>::Ptr pointcloud = kitti_reader.read_pointcloud(pointclouds_filepaths[i]);
                    pointcloud->header.frame_id = "";
                    pointcloud->header.seq = i;
                    pointcloud->header.stamp = timestamps[i];

                    skeleton.lidar_list[0].frontend.modules["prefilterer"]->update_raw_pointcloud_observer(pointcloud);
                    usleep(100000);
                }

                sleep(5);

                skeleton.backend.backend_handler->save_results(results_path);

                std::cout << "[END] Size: " << pointclouds_filepaths.size() << std::endl;
                std::cout << "[END] Prefilterer: " << skeleton.lidar_list[0].frontend.modules["prefilterer"]->total_time_ << " - " << skeleton.lidar_list[0].frontend.modules["prefilterer"]->count_ << std::endl;
                std::cout << "[END] Tracker: " << skeleton.lidar_list[0].frontend.modules["tracker"]->total_time_ << " - " << skeleton.lidar_list[0].frontend.modules["tracker"]->count_ << std::endl;
                std::cout << "[END] GroundDetector: " << skeleton.lidar_list[0].frontend.modules["ground_detector"]->total_time_ << " - " << skeleton.lidar_list[0].frontend.modules["ground_detector"]->count_ << std::endl;
                std::cout << "[END] LoopDetector: " << skeleton.loop_detector_list[0].loop_detector->total_time_ << " - " << skeleton.loop_detector_list[0].loop_detector->count_ << std::endl;
                std::cout << "[END] GraphHandler: " << skeleton.backend.graph_handler->total_time_ << " - " << skeleton.backend.graph_handler->count_ << std::endl;
                std::cout << "[END] BackendHandler: " << skeleton.backend.backend_handler->total_time_ << " - " << skeleton.backend.backend_handler->count_ << std::endl;

                return true;
            }

            /**
             *
             * @param path
             * @param extension
             * @param filepaths
             */
            void Controller::get_filepaths(const std::string &path, const std::string &extension,
                                                  std::vector<std::string> &filepaths) {
                for (const auto &p: std::filesystem::directory_iterator(path)) {
                    if (p.is_regular_file()) {
                        if (p.path().extension().string() == extension) {
                            filepaths.emplace_back(p.path());
                        }
                    }
                }

                std::sort(std::execution::par_unseq, filepaths.begin(), filepaths.end());
            }

            PLUGINLIB_EXPORT_CLASS(artslam::lots::wrapper::Controller, nodelet::Nodelet);
        }
    }
}
