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
#include "kernel/loop_detector.h"

namespace artslam
{
    namespace lots
    {
        namespace wrapper
        {
            /**
             * Initialize loop detector sub-modules.
             *
             * @param config_file
             */
            void LoopDetector::start(std::string config_file)
            {
                std::string welcome = " METHOD #" + std::to_string(_id) + " ";
                std::string title(100, '=');
                title.replace((int)(title.length() / 2 - (int)(welcome.length() / 2)), welcome.length(), welcome);
                std::cout << std::endl << title << std::endl;

                /* loop detector */
                if (_name == "full_lidar") {
                    Registration::Configuration loop_detector_registration_configuration =
                            parse_registration_loop_detector_configuration(config_file, 0);
                    LidarRegistration loop_detector_registration(loop_detector_registration_configuration);
                    artslam::lots::LoopDetector::Configuration loop_detector_configuration =
                            parse_loop_detector_configuration(config_file, 0);
                    loop_detector = std::make_shared<FullLidar>(loop_detector_configuration,
                                                                loop_detector_registration.registration_method());
                } else if (_name == "deep_radar") {
                    std::cout << "Deep Radar loop detector not implemented yet!" << std::endl;
                    loop_detector = nullptr;
                } else if (_name == "feature_camera") {
                    std::cout << "Feature Camera loop detector not implemented yet!" << std::endl;
                    loop_detector = nullptr;
                } else {
                    std::cout << "This loop detector name does not exist and it is not implemented, please choose among: 'full_lidar', 'deep_radar', 'feature_camera'." << std::endl;
                    loop_detector = nullptr;
                }
            }
        }
    }
}
