// -*- lots-wrapper-c++ -*-

/* ---------------------------------------------------------------------------------------------------------------------
 * Package: LOTS-WRAPPER
 * Class: Radar
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
#ifndef WRAPPER_RADAR_H
#define WRAPPER_RADAR_H

#include "sensor.h"

// ROS messages
#include <sensor_msgs/PointCloud2.h>

// PCL libraries
#include <pcl_conversions/pcl_conversions.h>


namespace artslam
{
    namespace lots
    {
        namespace wrapper
        {
            /**
             * Radar
             *
             * ...
             */
            class Radar : public Sensor<sensor_msgs::PointCloud2ConstPtr>  // TODO: change msg type
            {
                public:
                    Radar(int id, std::string topic, int buffer)
                    {
                        _prefilterer = true;
                        _tracker = true;
                        _ground_detector = true;

                        _sensor_type = "RADAR";
                        _sensor_id = id;

                        _start_color = "\033[1;36m";
                        _end_color = "\033[0m";

                        _topic = topic;
                        _buffer = buffer;
                    };

                    /**
                     * Method in charge to initialize the sensor ROS subscriber.
                     *
                     * @param mt_nh ROS Node Handler reference
                     */
                    void setSubscriber(ros::NodeHandle* mt_nh)
                    {
                        sensor_sub = mt_nh->subscribe(_topic, _buffer, &artslam::lots::wrapper::Radar::callback, this);
                    };

                    /**
                     * Sensor callback for the subscriber interaction.
                     *
                     * @param msg Template message which depends from the sensor type.
                     */
                    void callback(const sensor_msgs::PointCloud2ConstPtr& msg) override
                    {
                        ;
                    };
            };
        }
    }
}

#endif // WRAPPER_RADAR_H
