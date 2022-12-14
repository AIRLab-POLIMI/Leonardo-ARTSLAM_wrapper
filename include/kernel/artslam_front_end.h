// -*- artslam-c++ -*-

/* ---------------------------------------------------------------------------------------------------------------------
 * Package: ARTSLAM-WRAPPER
 * Class: ARTSLAMFrontEnd
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
#ifndef ARTSLAM_FRONT_END_H
#define ARTSLAM_FRONT_END_H

// ARTSLAM libraries
#include <configuration_parser.h>
#include "registration.h"
#include "tracker.h"
#include "ground_detector.h"
#include <artslam_io/pointcloud_io.h>
#include <prefilterer.h>

namespace artslam
{
    namespace laser3d
    {
        class ARTSLAMFrontEnd
        {
            public:
                std::shared_ptr<Prefilterer> prefilterer;
                std::shared_ptr<Tracker> tracker;
                std::shared_ptr<GroundDetector> ground_detector;

                ARTSLAMFrontEnd(){};

                void start(std::string config_file, bool _prefilterer, bool _tracker, bool _ground_detector);
        };
    }
}

#endif // ARTSLAM_FRONT_END_H
