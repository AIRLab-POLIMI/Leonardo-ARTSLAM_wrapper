// -*- artslam-c++ -*-

/* ---------------------------------------------------------------------------------------------------------------------
 * Package: ARTSLAM-WRAPPER
 * Class: ARTSLAMController
 * Author: Mirko Usuelli
 * Advisor: Prof. Matteo Matteucci, PhD
 * Co-Advisors: Matteo Frosi, PhD; Gianluca Bardaro, PhD; Simone Mentasti, PhD; Paolo Cudrano, PhD Student.
 * University: Politecnico di Milano - Artificial Intelligence & Robotics Lab
 * Last Modified: 30/11/2022
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
#ifndef ARTSLAM_SENSOR_H
#define ARTSLAM_SENSOR_H

#include "artslam_kernel.h"

// ROS libraries
#include <ros/ros.h>
#include <ros/subscriber.h>

namespace artslam
{
    namespace laser3d
    {
        /**
         * ARTSLAMSensor
         *
         * ...
         */
        template <class SensorMsgType>
        class ARTSLAMSensor
        {
            public:
                /* Attributes --------------------------------------------------------------------------------------- */
                ros::Subscriber sensor_sub;
                int counter = 0;
                ARTSLAMKernel* kernel;

                /* Methods ------------------------------------------------------------------------------------------ */
                ARTSLAMSensor(){};

                virtual void callback(const SensorMsgType& msg) = 0;
                void setKernel(ARTSLAMKernel* artslam_kernel) { kernel = artslam_kernel; };
        };
    }
}

#endif // ARTSLAM_SENSOR_H
