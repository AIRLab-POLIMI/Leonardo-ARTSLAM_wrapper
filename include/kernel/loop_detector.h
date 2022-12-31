// -*- lots-wrapper-c++ -*-

/* ---------------------------------------------------------------------------------------------------------------------
 * Package: LOTS-WRAPPER
 * Class: LoopDetector
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
#ifndef WRAPPER_LOOP_DETECTOR_H
#define WRAPPER_LOOP_DETECTOR_H

// ARTSLAM libraries
#include <utils/configuration_parser.h>
#include <frontend/lidar/lidar_registration.h>
#include <loop_detectors/full_lidar.h>

namespace artslam
{
    namespace lots
    {
        namespace wrapper
        {
            /**
             * LoopDetector
             *
             * The loop detector is historically argued to belong in the back-end or in the front-end; this choice to not
             * include in none of them is due to the fact that we want to allow a modular architecture able to manage
             * also sensor fusion in loop detection sub-modules for future related works.
             */
            class LoopDetector
            {
                public:
                    int _id;
                    std::shared_ptr<FullLidar> loop_detector;

                    LoopDetector(int id){ _id = id; };
                    void start(std::string config_file);
            };
        }
    }
}

#endif // WRAPPER_LOOP_DETECTOR_H
