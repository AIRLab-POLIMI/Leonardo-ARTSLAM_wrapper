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
#ifndef ARTSLAM_LIDAR_H
#define ARTSLAM_LIDAR_H

#include "../artslam_sensor.h"

// ROS messages
#include <sensor_msgs/PointCloud2.h>

// PCL libraries
#include <pcl_conversions/pcl_conversions.h>


namespace artslam
{
    namespace laser3d
    {
        /**
         * ARTSLAMLidar
         *
         * ...
         */
        class ARTSLAMLidar //: public ARTSLAMSensor<sensor_msgs::PointCloud2ConstPtr>
        {
            public:
                /* Attributes --------------------------------------------------------------------------------------- */
                int counter = 0;
                ros::Subscriber sub;
                ARTSLAMKernel* kernel;

                /* Methods ------------------------------------------------------------------------------------------ */
                ARTSLAMLidar(){};

                void setKernel(ARTSLAMKernel* artslam_kernel)
                {
                    kernel = artslam_kernel;
                };

                void callback(const sensor_msgs::PointCloud2ConstPtr& msg)
                {
                    if(!ros::ok())
                        return;

                    pcl::PointCloud<Point3I>::Ptr cloud(new pcl::PointCloud<Point3I>());
                    pcl::fromROSMsg(*msg, *cloud);
                    cloud->header.seq = counter;

                    cloud->header.stamp = msg->header.stamp.toNSec();
                    kernel->prefilterer->update_raw_pointcloud_observer(cloud);

                    counter++;
                };
        };
    }
}

#endif // ARTSLAM_LIDAR_H
