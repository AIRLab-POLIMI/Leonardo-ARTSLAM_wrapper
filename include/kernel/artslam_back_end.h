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
#ifndef ARTSLAM_BACK_END_H
#define ARTSLAM_BACK_END_H

// ARTSLAM libraries
#include <configuration_parser.h>
#include "registration.h"
#include "graph_handler.h"
#include <backend_handler.h>
#include "information_matrix_calculator.h"
#include "controller/artslam_bridge_visualizer.h"

namespace artslam
{
    namespace laser3d
    {
        /**
         * ARTSLAMBackEnd
         *
         * Back-end module in charge of collecting ARTSLAM back-end submodules and initialize them. Each module is
         * unique in the whole system (contrary to the back-end).
         */
        class ARTSLAMBackEnd
        {
            public:
                std::shared_ptr<BackendHandler> backend_handler;
                std::shared_ptr<GraphHandler> graph_handler;
                std::shared_ptr<InformationMatrixCalculator> information_matrix_calculator;

                ARTSLAMBackEnd(){};

                void start(ARTSLAMBridgeVisualizer* bridge, std::string config_file);
        };
    }
}

#endif // ARTSLAM_BACK_END_H
