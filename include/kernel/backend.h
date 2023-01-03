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
#ifndef WRAPPER_BACKEND_H
#define WRAPPER_BACKEND_H

// LOTS-SLAM libraries
#include <utils/configuration_parser.h>
#include <sensors/lidar/lidar_registration.h>
#include <backend/graph_handler.h>
#include <backend/backend_handler.h>
#include <backend/information_matrix_calculator.h>
#include "../controller/bridge_visualizer.h"

namespace artslam
{
    namespace lots
    {
        namespace wrapper
        {
            /**
             * Backend
             *
             * Back-end module in charge of collecting ARTSLAM back-end submodules and initialize them. Each module is
             * unique in the whole system (contrary to the back-end).
             */
            class Backend
            {
                public:
                    std::shared_ptr<BackendHandler> backend_handler;
                    std::shared_ptr<GraphHandler> graph_handler;
                    std::shared_ptr<InformationMatrixCalculator> information_matrix_calculator;

                    Backend(){};

                    void start(BridgeVisualizer* bridge, std::string config_file);
            };
        }
    }
}

#endif // WRAPPER_BACKEND_H
