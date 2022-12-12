// Header library
#include "artslam_controller.h"
#include "sensors/artslam_lidar.h"
#include "sensors/artslam_imu.h"
#include "sensors/artslam_gnss.h"

#include <artslam_io/kitti_reader.hpp>
#include <filesystem>
#include <execution>

#include <sensor_msgs/PointCloud2.h>

#include <pluginlib/class_list_macros.h>

namespace artslam
{
    namespace laser3d
    {
        ARTSLAMController::ARTSLAMController() : param_value(0) { return; };

        void ARTSLAMController::onInit()
        {
            private_nh = getPrivateNodeHandle();
            mt_nh = getMTNodeHandle();
            bridge.set_handler(mt_nh);
            private_nh.getParam("value", param_value);

            private_nh.getParam("configuration_file", config_file);
            private_nh.getParam("results_path", results_path);

            startKernel(config_file, results_path);

            service = private_nh.advertiseService("OfflineSLAM", &ARTSLAMController::offline_slam, this);

            lidar.setKernel(&kernel);
            lidar.sub = mt_nh.subscribe("/velodyne_points", 64, &artslam::laser3d::ARTSLAMLidar::callback, &lidar);

            imu.setKernel(&kernel);
            imu.sub = mt_nh.subscribe("/imu_data", 1024, &artslam::laser3d::ARTSLAMImu::callback, &imu);

            gnss.setKernel(&kernel);
            gnss.sub = mt_nh.subscribe("/gnss_data", 1024, &artslam::laser3d::ARTSLAMGnss::callback, &gnss);

        }

        void ARTSLAMController::startKernel(std::string config_file, std::string results_path)
        {
            /* prefilterer */
            Prefilterer::Configuration prefilterer_configuration =
                    artslam::laser3d::parse_prefilterer_configuration(config_file);
            kernel.prefilterer = std::make_shared<Prefilterer>(prefilterer_configuration);

            /* tracker */
            Registration::Configuration registration_tracker_configuration =
                    parse_registration_tracker_configuration(config_file);
            Registration registration(registration_tracker_configuration);
            Tracker::Configuration tracker_configuration = parse_tracker_configuration(config_file);
            kernel.tracker = std::make_shared<Tracker>(tracker_configuration, registration.registration_method());

            /* ground detector */
            GroundDetector::Configuration ground_detector_configuration =
                    parse_ground_detector_configuration(config_file);
            kernel.ground_detector = std::make_shared<GroundDetector>(ground_detector_configuration);

            /* graph handler */
            kernel.graph_handler = std::make_shared<GraphHandler>();

            /* information matrix calculator */
            InformationMatrixCalculator::Configuration information_matrix_calculator_configuration =
                    parse_information_matrix_calculator(config_file);
            kernel.information_matrix_calculator =
                    std::make_shared<InformationMatrixCalculator>(information_matrix_calculator_configuration);

            /* loop detector */
            Registration::Configuration loop_detector_registration_configuration =
                    parse_registration_loop_detector_configuration(config_file);
            Registration loop_detector_registration(loop_detector_registration_configuration);
            LoopDetector::Configuration loop_detector_configuration =
                    parse_loop_detector_configuration(config_file);
            kernel.loop_detector = std::make_shared<LoopDetector>(loop_detector_configuration,
                                                                  loop_detector_registration.registration_method());

            /* backend handler */
            BackendHandler::Configuration backend_handler_configuration =
                    parse_backend_handler_configuration(config_file);
            kernel.backend_handler = std::make_shared<BackendHandler>(backend_handler_configuration);
            kernel.backend_handler->set_graph_handler(kernel.graph_handler.get());
            kernel.backend_handler->set_information_matrix_calculator(kernel.information_matrix_calculator.get());
            kernel.backend_handler->set_loop_detector(kernel.loop_detector.get());

            /* SLAM pipeline construction */
            kernel.prefilterer->register_filtered_pointcloud_observer(kernel.tracker.get());
            kernel.prefilterer->register_filtered_pointcloud_observer(kernel.ground_detector.get());
            kernel.tracker->register_keyframe_observer(kernel.backend_handler.get());
            kernel.ground_detector->register_floor_coefficients_observer(kernel.backend_handler.get());
            kernel.backend_handler->register_slam_output_observer(&bridge);
        }

        bool ARTSLAMController::offline_slam(
                artslam_wrapper::OfflineSLAM::Request &req,
                artslam_wrapper::OfflineSLAM::Request &res)
        {
            //kernel.backend_handler->save_results(results_path);
            //return true;

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

                kernel.prefilterer->update_raw_pointcloud_observer(pointcloud);
                usleep(100000);
            }

            sleep(5);

            kernel.backend_handler->save_results(results_path);

            std::cout << "[END] Size: " << pointclouds_filepaths.size() << std::endl;
            std::cout << "[END] Prefilterer: " << kernel.prefilterer->total_time_ << " - " << kernel.prefilterer->count_ << std::endl;
            std::cout << "[END] Tracker: " << kernel.tracker->total_time_ << " - " << kernel.tracker->count_ << std::endl;
            std::cout << "[END] GroundDetector: " << kernel.ground_detector->total_time_ << " - " << kernel.ground_detector->count_ << std::endl;
            std::cout << "[END] LoopDetector: " << kernel.loop_detector->total_time_ << " - " << kernel.loop_detector->count_ << std::endl;
            std::cout << "[END] GraphHandler: " << kernel.graph_handler->total_time_ << " - " << kernel.graph_handler->count_ << std::endl;
            std::cout << "[END] BackendHandler: " << kernel.backend_handler->total_time_ << " - " << kernel.backend_handler->count_ << std::endl;

            return true;
        }

        void ARTSLAMController::get_filepaths(const std::string &path, const std::string &extension,
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

        PLUGINLIB_EXPORT_CLASS(artslam::laser3d::ARTSLAMController, nodelet::Nodelet);
    }
}
