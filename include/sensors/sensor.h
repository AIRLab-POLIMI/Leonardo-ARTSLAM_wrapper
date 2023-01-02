// -*- lots-wrapper-c++ -*-

/* ---------------------------------------------------------------------------------------------------------------------
 * Package: LOTS-WRAPPER
 * Class: Sensor
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
#ifndef WRAPPER_SENSOR_H
#define WRAPPER_SENSOR_H

#include "kernel/skeleton.h"

// ROS libraries
#include <ros/ros.h>
#include <ros/subscriber.h>
#include <boost/algorithm/string.hpp>

// TF libraries
#include <tf/transform_listener.h>

namespace artslam
{
    namespace lots
    {
        namespace wrapper
        {
            /**
             * ARTSLAMSensor
             *
             * Abstract class which standardizes the concept of a sensor to be used with ARTSLAM.
             */
            template <class SensorMsgType>
            class Sensor
            {
                protected:
                    /* Attributes ----------------------------------------------------------------------------------- */
                    bool _prefilterer;
                    bool _tracker;
                    bool _ground_detector;

                    std::string _sensor_type;
                    int _sensor_id;

                    std::string _topic;
                    int _buffer;
                    tf::TransformListener* tf_listener;

                public:
                    /* Attributes ----------------------------------------------------------------------------------- */
                    ros::Subscriber sensor_sub;
                    int counter = 0;

                    std::string _start_color;
                    std::string _end_color;

                    Frontend frontend;
                    LoopDetector* loop_detector;
                    Backend* backend;

                    /* Methods -------------------------------------------------------------------------------------- */
                    Sensor(){};

                    /**
                     * Sensor callback for the subscriber interaction.
                     *
                     * @param msg Template message which depends from the sensor type.
                     */
                    virtual void callback(const SensorMsgType& msg) = 0;

                    /**
                     * Method in charge to initialize the sensor ROS subscriber.
                     *
                     * @param mt_nh ROS Node Handler reference
                     */
                    virtual void setSubscriber(ros::NodeHandle* mt_nh) = 0;

                    /**
                     * Method which initialize the front-end related to the current sensor by plugging the back-end and
                     * enabling the subscriber to work with the ROS node.
                     *
                     * @param mt_nh ROS node handler
                     * @param artslam_backend Back-End reference
                     * @param config_file Configuration file
                     */
                    void start(ros::NodeHandle* mt_nh, Backend* backend, std::string config_file)
                    {
                        std::string welcome = " " + _sensor_type + " #" + std::to_string(_sensor_id) + " ";
                        std::string title(100, '=');
                        title.replace((int)(title.length() / 2 - (int)(welcome.length() / 2)), welcome.length(), welcome);

                        /* front-end */
                        std::cout << _start_color << std::endl;
                        std::cout << title << std::endl;
                        std::cout << "- ROS topic name: " << _topic << std::endl;
                        std::cout << "- ROS buffer size: " << _buffer << std::endl;
                        std::string sensor_type =  boost::algorithm::to_lower_copy(_sensor_type);
                        frontend.start(config_file, sensor_type, _sensor_id);
                        std::cout << _end_color;

                        /* back-end */
                        setBackEnd(backend);

                        /* ROS-Subscribers */
                        setSubscriber(mt_nh);
                    }

                    /**
                     * Append a new reference to a loop detector which involves the current sensor.
                     *
                     * @param artslam_loop_detector Reference to a Loop Detector module.
                     */
                    void addLoopDetector(LoopDetector* _loop_detector)
                    {
                        // TODO: add a vector of references
                        loop_detector = _loop_detector;
                    };

                    /**
                     * Store the back-end reference to be used by the sensor and plug front-end modules with the back-end.
                     *
                     * @param artslam_backend Back-end reference
                     */
                    void setBackEnd(Backend* _backend)
                    {
                        backend = _backend;

                        if (frontend.modules.find("tracker") != frontend.modules.end())
                            frontend.modules["tracker"]->register_keyframe_observer(backend->backend_handler.get());

                        if (frontend.modules.find("ground_detector") != frontend.modules.end())
                            frontend.modules["ground_detector"]->register_floor_coefficients_observer(backend->backend_handler.get());
                    };
            };
        }
    }
}

#endif // WRAPPER_SENSOR_H
