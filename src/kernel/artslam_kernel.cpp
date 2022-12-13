// Header library

#include "artslam_kernel.h"

namespace artslam
{
    namespace laser3d
    {
        void ARTSLAMKernel::start(std::string config_file, std::string results_path)
        {
            /* prefilterer */
            Prefilterer::Configuration prefilterer_configuration =
                    artslam::laser3d::parse_prefilterer_configuration(config_file);
            prefilterer = std::make_shared<Prefilterer>(prefilterer_configuration);

            /* tracker */
            Registration::Configuration registration_tracker_configuration =
                    parse_registration_tracker_configuration(config_file);
            Registration registration(registration_tracker_configuration);
            Tracker::Configuration tracker_configuration = parse_tracker_configuration(config_file);
            tracker = std::make_shared<Tracker>(tracker_configuration, registration.registration_method());

            /* ground detector */
            GroundDetector::Configuration ground_detector_configuration =
                    parse_ground_detector_configuration(config_file);
            ground_detector = std::make_shared<GroundDetector>(ground_detector_configuration);

            /* graph handler */
            graph_handler = std::make_shared<GraphHandler>();

            /* information matrix calculator */
            InformationMatrixCalculator::Configuration information_matrix_calculator_configuration =
                    parse_information_matrix_calculator(config_file);
            information_matrix_calculator =
                    std::make_shared<InformationMatrixCalculator>(information_matrix_calculator_configuration);

            /* loop detector */
            Registration::Configuration loop_detector_registration_configuration =
                    parse_registration_loop_detector_configuration(config_file);
            Registration loop_detector_registration(loop_detector_registration_configuration);
            LoopDetector::Configuration loop_detector_configuration =
                    parse_loop_detector_configuration(config_file);
            loop_detector = std::make_shared<LoopDetector>(loop_detector_configuration,
                                                                  loop_detector_registration.registration_method());

            /* backend handler */
            BackendHandler::Configuration backend_handler_configuration =
                    parse_backend_handler_configuration(config_file);
            backend_handler = std::make_shared<BackendHandler>(backend_handler_configuration);
            backend_handler->set_graph_handler(graph_handler.get());
            backend_handler->set_information_matrix_calculator(information_matrix_calculator.get());
            backend_handler->set_loop_detector(loop_detector.get());

            /* SLAM pipeline construction */
            prefilterer->register_filtered_pointcloud_observer(tracker.get());
            prefilterer->register_filtered_pointcloud_observer(ground_detector.get());
            tracker->register_keyframe_observer(backend_handler.get());
            ground_detector->register_floor_coefficients_observer(backend_handler.get());
        }
    }
}
