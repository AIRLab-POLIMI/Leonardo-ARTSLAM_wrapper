// -*- artslam-c++ -*-

/* ---------------------------------------------------------------------------------------------------------------------
 * Package: ARTSLAM-WRAPPER
 * Class: ARTSLAMLoopDetector
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
#ifndef ARTSLAM_LOOP_DETECTOR_H
#define ARTSLAM_LOOP_DETECTOR_H

// ARTSLAM libraries
#include <configuration_parser.h>
#include "registration.h"

namespace artslam
{
    namespace laser3d
    {
        /**
         * ARTSLAMLoopDetector
         *
         * The loop detector is historically argued to belong in the back-end or in the front-end; this choice to not
         * include in none of them is due to the fact that we want to allow a modular architecture able to manage
         * also sensor fusion in loop detection sub-modules for future related works.
         */
        class ARTSLAMLoopDetector
        {
            public:
                int _id;
                std::shared_ptr<LoopDetector> loop_detector;

                ARTSLAMLoopDetector(int id){ _id = id; };
                void start(std::string config_file);
        };
    }
}

#endif // ARTSLAM_LOOP_DETECTOR_H
