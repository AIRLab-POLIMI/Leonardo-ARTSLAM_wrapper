// Header library

#include "kernel/artslam_back_end.h"

namespace artslam
{
    namespace laser3d
    {
        void ARTSLAMBackEnd::start(ARTSLAMBridgeVisualizer* bridge, std::string config_file)
        {
            /* graph handler */
            graph_handler = std::make_shared<GraphHandler>();

            /* information matrix calculator */
            InformationMatrixCalculator::Configuration information_matrix_calculator_configuration =
                    parse_information_matrix_calculator(config_file);
            information_matrix_calculator =
                    std::make_shared<InformationMatrixCalculator>(information_matrix_calculator_configuration);

            /* backend handler */
            BackendHandler::Configuration backend_handler_configuration =
                    parse_backend_handler_configuration(config_file);
            backend_handler = std::make_shared<BackendHandler>(backend_handler_configuration);
            backend_handler->set_graph_handler(graph_handler.get());
            backend_handler->set_information_matrix_calculator(information_matrix_calculator.get());
            backend_handler->register_slam_output_observer(bridge);
        }
    }
}
