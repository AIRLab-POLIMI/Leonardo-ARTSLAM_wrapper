// -*- lots-wrapper-c++ -*-

/* ---------------------------------------------------------------------------------------------------------------------
 * Package: LOTS-WRAPPER
 * Class: Gnss
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
#ifndef WRAPPER_GNSS_H
#define WRAPPER_GNSS_H

#include "sensor.h"

// ROS messages
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>

#include <tf_conversions/tf_eigen.h>

namespace artslam
{
    namespace lots
    {
        namespace wrapper
        {
            /**
             * Gnss
             *
             * Sensor class casted to manage GNSS behaviour.
             */
            class Gnss : public Sensor<sensor_msgs::NavSatFixConstPtr>
            {
                public:
                    Gnss(tf::TransformListener* _tf_listener, int id, std::string topic, int buffer)
                    {
                        _sensor_type = "GNSS";
                        _sensor_id = id;

                        _start_color = "\033[1;35m";
                        _end_color = "\033[0m";

                        _topic = topic;
                        _buffer = buffer;
                        tf_listener = _tf_listener;
                    };

                    /**
                     * Method in charge to initialize the sensor ROS subscriber.
                     *
                     * @param mt_nh ROS Node Handler reference
                     */
                    void setSubscribers(ros::NodeHandle* mt_nh)
                    {
                        sensor_sub = mt_nh->subscribe(_topic, _buffer, &artslam::lots::wrapper::Gnss::callback, this);
                    };

                    /**
                     * Sensor callback for the subscriber interaction.
                     *
                     * @param msg Template message which depends from the sensor type.
                     */
                    void callback(const sensor_msgs::NavSatFixConstPtr& msg) override
                    {
                        if(counter == 0)
                        {
                            tf::StampedTransform transform;
                            try
                            {
                                tf_listener->lookupTransform("base_link", msg->header.frame_id, ros::Time::now(), transform);
                            }
                            catch (std::exception &e)
                            {
                                std::cerr << "failed to find gps transform!!" << std::endl;
                                return;
                            }

                            EigVector3d g2l_translation(
                                    transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
                            backend->backend_handler->set_gps_to_lidar_translation(g2l_translation);
                        }

                        GeoPointStamped_MSG::Ptr conv_gnss_msg(new GeoPointStamped_MSG);
                        conv_gnss_msg->header_.timestamp_ = msg->header.stamp.toNSec();
                        conv_gnss_msg->header_.frame_id_ = "base_link";
                        conv_gnss_msg->header_.sequence_ = counter;
                        counter++;
                        conv_gnss_msg->latitude_ = msg->latitude;
                        conv_gnss_msg->longitude_ = msg->longitude;
                        conv_gnss_msg->altitude_ = msg->altitude;
                        conv_gnss_msg->covariance_type_ = msg->position_covariance_type;

                        for(int i = 0; i < 9; i++)
                        {
                            if(conv_gnss_msg->covariance_type_ == 0)
                            {
                                conv_gnss_msg->covariance_[i] = 0.0;
                            }
                            else
                            {
                                conv_gnss_msg->covariance_[i] = msg->position_covariance[i];
                            }
                        }
                        backend->backend_handler->update_raw_gnss_observer(conv_gnss_msg);
                    };
            };
        }
    }
}

#endif // WRAPPER_GNSS_H
