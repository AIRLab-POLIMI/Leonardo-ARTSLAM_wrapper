// -*- lots-wrapper-c++ -*-

/* ---------------------------------------------------------------------------------------------------------------------
 * Package: LOTS-WRAPPER
 * Class: Frontend
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
#include "kernel/frontend.h"

namespace artslam
{
    namespace lots
    {
        namespace wrapper
        {
            /**
             * Initialize front-end sub-modules based on flag configuration which strongly depends from the sensor type.
             *
             * @param config_file Configuration file.
             * @param _prefilterer Prefilterer flag
             * @param _tracker Tracker flag
             * @param _ground_detector Ground detector flag
             */
            void Frontend::start(std::string config_file, bool _prefilterer, bool _tracker, bool _ground_detector)
            {
                /* prefilterer */
                if (_prefilterer)
                {
                    std::cout << ">> PRE-FILTERER" << std::endl;
                    Prefilterer::Configuration prefilterer_configuration = parse_prefilterer_configuration(config_file, "lidar", 0);
                    prefilterer = std::make_shared<Prefilterer>(prefilterer_configuration);

                    std::cout << std::endl;
                }
                else
                {
                    prefilterer = nullptr;
                }

                /* tracker */
                if (_tracker)
                {
                    std::cout << ">> TRACKER" << std::endl;
                    Registration::Configuration registration_tracker_configuration = parse_registration_tracker_configuration(config_file, "lidar", 0);
                    Registration registration(registration_tracker_configuration);
                    Tracker::Configuration tracker_configuration = parse_tracker_configuration(config_file, "lidar", 0);
                    tracker = std::make_shared<Tracker>(tracker_configuration, registration.registration_method());

                    if (_prefilterer)
                    {
                        prefilterer->register_filtered_pointcloud_observer(tracker.get());
                    }

                    std::cout << std::endl;
                }
                else
                {
                    tracker = nullptr;
                }

                /* ground detector */
                if (_ground_detector)
                {
                    std::cout << ">> GROUND-DETECTOR" << std::endl;
                    GroundDetector::Configuration ground_detector_configuration =
                            parse_ground_detector_configuration(config_file, "lidar", 0);
                    ground_detector = std::make_shared<GroundDetector>(ground_detector_configuration);

                    if (_ground_detector)
                    {
                        prefilterer->register_filtered_pointcloud_observer(ground_detector.get());
                    }

                    std::cout << std::endl;
                }
                else
                {
                    ground_detector = nullptr;
                }
            }
        }
    }
}
