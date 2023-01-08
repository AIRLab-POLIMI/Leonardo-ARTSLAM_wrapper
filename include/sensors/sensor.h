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
#include <nav_msgs/Odometry.h>

// TF libraries
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

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
                    std::string _sensor_type;
                    int _sensor_id;

                    std::string _topic;
                    std::string _prior_odom_topic = "/0";
                    int _buffer;
                    tf::TransformListener* tf_listener;

                public:
                    /* Attributes ----------------------------------------------------------------------------------- */
                    ros::Subscriber sensor_sub;
                    ros::Subscriber prior_odom_sub;
                    int counter = 0;

                    std::string _start_color;
                    std::string _end_color;

                    Frontend frontend;
                    LoopDetector* loop_detector;
                    Backend* backend;

                    /* Methods -------------------------------------------------------------------------------------- */
                    Sensor(){};

                    void set_prior_odom_topic(std::string topic) {
                        _prior_odom_topic = topic;
                    }

                    /**
                     * Sensor callback for the subscriber interaction.
                     *
                     * @param msg Template message which depends from the sensor type.
                     */
                    virtual void callback(const SensorMsgType& msg) = 0;

                    /**
                     * Sensor callback for the prior odom topic stram.
                     *
                     * @param msg Odometry message.
                     */
                    void prior_odom_callback(const nav_msgs::OdometryConstPtr& msg) {};

                    /**
                     * Method in charge to initialize the sensor ROS subscriber.
                     *
                     * @param mt_nh ROS Node Handler reference
                     */
                    virtual void setSubscribers(ros::NodeHandle* mt_nh) = 0;

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
                        std::string sensor_type = boost::algorithm::to_lower_copy(_sensor_type);
                        std::string welcome = " " + _sensor_type + " #" + std::to_string(_sensor_id) + " ";
                        std::string title(100, '=');
                        title.replace((int)(title.length() / 2 - (int)(welcome.length() / 2)), welcome.length(), welcome);

                        /* front-end */
                        std::cout << _start_color << std::endl;
                        std::cout << title << std::endl;
                        std::cout << "- ROS topic name: " << _topic << std::endl;
                        if (_prior_odom_topic != "/0") {
                            std::cout << "- ROS prior odom topic name: " << _prior_odom_topic << std::endl;
                        }
                        std::cout << "- ROS buffer size: " << _buffer << std::endl;
                        std::cout << "- T(" << sensor_type << "_" << std::to_string(_sensor_id) << "->base)   [ R | t ] " << std::endl;
                        std::cout << backend->backend_handler->converter.get_sensor2base_tf(sensor_type, _sensor_id).matrix() << std::endl;
                        std::cout << std::endl;

                        frontend.start(config_file, sensor_type, _sensor_id);

                        /* back-end */
                        setBackEnd(backend);

                        /* ROS-Subscribers */
                        setSubscribers(mt_nh);

                        std::cout << _end_color;
                        std::cout << std::endl;
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
                    void setBackEnd(Backend* _backend) {
                        backend = _backend;
                        std::string sensor_type = boost::algorithm::to_lower_copy(_sensor_type);

                        if (frontend.modules.find("tracker") != frontend.modules.end()){
                            if (sensor_type == "lidar")
                                (static_cast<LidarTracker*>(frontend.modules["tracker"].get()))->register_keyframe_observer(
                                        backend->backend_handler.get());
                            else if (sensor_type == "odom")
                                (static_cast<OdomTracker*>(frontend.modules["tracker"].get()))->register_keyframe_observer(
                                        backend->backend_handler.get());
                            std::cout << "[BackendHandler] Registered to the Tracker!" << std::endl;
                        }

                        if (frontend.modules.find("ground_detector") != frontend.modules.end()) {
                            (static_cast<LidarGroundDetector*>(frontend.modules["ground_detector"].get()))->register_floor_coefficients_observer(
                                    backend->backend_handler.get());
                            std::cout << "[BackendHandler] Registered to the GroundDetector!" << std::endl;
                        }
                    };
            };
        }
    }
}

#endif // WRAPPER_SENSOR_H
