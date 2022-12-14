// -*- artslam-c++ -*-

/* ---------------------------------------------------------------------------------------------------------------------
 * Package: ARTSLAM-WRAPPER
 * Class: ARTSLAMController
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
#ifndef ARTSLAM_SENSOR_H
#define ARTSLAM_SENSOR_H

#include "kernel/artslam_kernel.h"

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
            protected:
                bool _prefilterer;
                bool _tracker;
                bool _ground_detector;
                std::string _topic;
                int _buffer;

            public:
                /* Attributes --------------------------------------------------------------------------------------- */
                ros::Subscriber sensor_sub;
                int counter = 0;

                ARTSLAMFrontEnd frontend;
                ARTSLAMLoopDetector* loop_detector;
                ARTSLAMBackEnd* backend;

                /* Methods ------------------------------------------------------------------------------------------ */
                ARTSLAMSensor(){};

                virtual void callback(const SensorMsgType& msg) = 0;

                virtual void setSubscriber(ros::NodeHandle* mt_nh) = 0;

                void start(ros::NodeHandle* mt_nh, ARTSLAMBackEnd* artslam_backend, std::string config_file)
                {
                    /* front-end */
                    frontend.start(config_file, _prefilterer, _tracker, _ground_detector);

                    /* back-end */
                    setBackEnd(artslam_backend);

                    /* ROS-Subscribers */
                    setSubscriber(mt_nh);
                }

                void addLoopDetector(ARTSLAMLoopDetector* artslam_loop_detector)
                {
                    loop_detector = artslam_loop_detector;
                };

                void setBackEnd(ARTSLAMBackEnd* artslam_backend)
                {
                    backend = artslam_backend;

                    if (_tracker)
                    {
                        frontend.tracker->register_keyframe_observer(backend->backend_handler.get());
                    }

                    if (_ground_detector)
                    {
                        frontend.ground_detector->register_floor_coefficients_observer(backend->backend_handler.get());
                    }
                };
        };
    }
}

#endif // ARTSLAM_SENSOR_H
