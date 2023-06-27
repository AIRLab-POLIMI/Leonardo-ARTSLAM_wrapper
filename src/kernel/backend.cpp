// -*- lots-wrapper-c++ -*-

/* ---------------------------------------------------------------------------------------------------------------------
 * Package: LOTS-WRAPPER
 * Class: Backend
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
#include "kernel/backend.hpp"

namespace lots::slam::wrapper
        {
    using namespace lots::slam::utils;
            /**
             * Initialize the back-end.
             *
             * @param bridge BridgeVisualizer reference
             * @param config_file Configuration file
             */
            void Backend::start(BridgeVisualizer* bridge, std::string config_file)
            {
                /* graph handler ------------------------------------------------------------------------------------ */
                std::cout << ">> GRAPH-HANDLER" << std::endl;
                graph_handler = std::make_shared<GraphHandler>();


                /* information matrix calculator ---------------------------------------------------------------------*/
//                std::cout << std::endl << ">> INFORMATION-MATRIX-CALCULATOR" << std::endl;
//                InformationMatrixCalculator::Configuration im_config = parse_information_matrix_calculator(config_file);
//                information_matrix_calculator = std::make_shared<InformationMatrixCalculator>(im_config);

                /* backend handler ---------------------------------------------------------------------------------- */
                std::cout << std::endl << ">> BACKEND-HANDLER" << std::endl;
                BackendHandler::Configuration bh_config = parse_backend_handler_configuration(config_file);
                backend_handler = std::make_shared<BackendHandler>(bh_config);
                backend_handler->set_converter(config_file);

                // building up everything together
                backend_handler->set_graph_handler(graph_handler.get());
//                backend_handler->set_information_matrix_calculator(information_matrix_calculator.get());
                backend_handler->register_slam_output_observer(bridge);
            }
        }
