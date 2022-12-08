// Header library
#include "../../include/artslam_controller.h"
#include "../../include/sensors/artslam_lidar.h"
#include "../../include/sensors/artslam_imu.h"
#include "../../include/sensors/artslam_gnss.h"

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
                artslam_laser_3d_wrapper::OfflineSLAM::Request &req,
                artslam_laser_3d_wrapper::OfflineSLAM::Request &res)
        {
            kernel.backend_handler->save_results(results_path);
            return true;
        }

        PLUGINLIB_EXPORT_CLASS(artslam::laser3d::ARTSLAMController, nodelet::Nodelet);
    }
}
