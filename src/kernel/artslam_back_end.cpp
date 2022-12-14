// -*- artslam-c++ -*-

/* ---------------------------------------------------------------------------------------------------------------------
 * Package: ARTSLAM-WRAPPER
 * Class: ARTSLAMBackEnd
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
#include "kernel/artslam_back_end.h"

namespace artslam
{
    namespace laser3d
    {
        /**
         * Initialize the back-end.
         *
         * @param bridge BridgeVisualizer reference
         * @param config_file Configuration file
         */
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
