// -*- lots-wrapper-c++ -*-

/* ---------------------------------------------------------------------------------------------------------------------
 * Package: LOTS-WRAPPER
 * Class: BridgeVisualizer
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

// Header library
#include "controller/bridge_visualizer.h"

// ROS Messages libraries
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>

// TFs libraries
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

// PCL libraries
#include <pcl_conversions/pcl_conversions.h>


namespace artslam::lots::wrapper
{
    /**
     * Initialize the bridge-visualizer for ARTSLAM-ROS functionalities.
     */
    BridgeVisualizer::BridgeVisualizer() : tf_listener(tf_buffer)
    {
        main_reference = "base_link";

        // Dispatcher initialization for the visualizer
        dispatcher = std::make_unique<core::utils::Dispatcher>(MODULE_ID, 1);

        // ROS Publishers initialization
        markers_pub = handler.advertise<visualization_msgs::MarkerArray>(MARKER_TOPIC, 16);
        pointcloud_pub = handler.advertise<sensor_msgs::PointCloud2>(POINTCLOUD_TOPIC, 1);
        pose_pub = handler.advertise<geometry_msgs::PoseStamped>(POSE_TOPIC, 10);
        occgrid_map_pub = handler.advertise<nav_msgs::OccupancyGrid>(OCCUPANCYGRID_TOPIC, 1);
        tf_pub = handler.createTimer(ros::Duration(0.1), &BridgeVisualizer::timer_callback, this);

        // Initial TF configuration message map->odom
        geometry_msgs::TransformStamped init_msg;
        init_msg.header.stamp = ros::Time::now();
        init_msg.header.frame_id = "map";
        init_msg.child_frame_id = "odom";
        init_msg.transform.translation.x = init_msg.transform.translation.y = init_msg.transform.translation.z = 0;
        init_msg.transform.rotation.x = init_msg.transform.rotation.y = init_msg.transform.rotation.z = 0;
        init_msg.transform.rotation.w = 1;
        tf_broadcaster.sendTransform(init_msg);
    }

    void BridgeVisualizer::timer_callback(const ros::TimerEvent& event) {
        if(!latest_transform.header.frame_id.empty()) {
            latest_transform.header.stamp = ros::Time::now() + ros::Duration(0.2);
            tf_broadcaster.sendTransform(latest_transform);
        }
    }

    /**
     * It signals that a new filtered point cloud has been received and it is going to be displayed.
     *
     * @param pointcloud Filtered PCL pointcloud to be displayed.
     */
    void BridgeVisualizer::update_filtered_pointcloud_observer(pcl::PointCloud<Point3I>::ConstPtr pointcloud)
    {
        // sending the point cloud to the visualizer through the dispatcher
        dispatcher->dispatch([this, pointcloud]{draw_pointcloud(pointcloud);});
    }

    /**
     * It signals that map and poses are ready to be displayed.
     *
     * @param map Map to be displayed.
     * @param poses Poses to be displayed.
     */
    void BridgeVisualizer::update_slam_output_observer(
            pcl::PointCloud<Point3I>::Ptr map,
            std::vector<Eigen::Isometry3d> poses,
            OccupancyGrid::Ptr occupancy_grid)
    {
        // sending map and poses to the visualizer through the dispatcher
        std::cout << "update_slam_output_observer" << std::endl;
        dispatcher->dispatch([this, map, poses, occupancy_grid]{draw_map_and_poses(map, poses, occupancy_grid);});
    }

    /**
     * It draws a new received pointcloud.
     *
     * @param pointcloud Filtered PCL pointcloud to be displayed.
     */
    void BridgeVisualizer::draw_pointcloud(const pcl::PointCloud<Point3I>::ConstPtr& pointcloud)
    {
        // pointcloud ros message
        sensor_msgs::PointCloud2Ptr pointcloud_msg(new sensor_msgs::PointCloud2());

        // pointcloud pcl
        pcl::PointCloud<Point3I>::Ptr pointcloud_pcl(new pcl::PointCloud<Point3I>);

        // point cloud publishing
        *pointcloud_pcl = *pointcloud;
        pointcloud_pcl->header.stamp /= 1000ull;
        pcl::toROSMsg(*pointcloud_pcl, *pointcloud_msg);
        pointcloud_msg->header.frame_id = main_reference;
        pointcloud_pub.publish(pointcloud_msg);
    }

    /**
     * Signals that map and poses are ready to be displayed.
     *
     * @param map Map to be displayed.
     * @param poses Poses to be displayed.
     */
    void BridgeVisualizer::draw_map_and_poses(pcl::PointCloud<Point3I>::Ptr map,
                                                     std::vector<EigIsometry3d> poses,
                                                     OccupancyGrid::Ptr occupancy_grid)
    {
        // pointcloud message
        sensor_msgs::PointCloud2Ptr pointcloud_msg(new sensor_msgs::PointCloud2());
        ros::Time stamp;
        stamp.sec = map->header.stamp / 1000000000ull;
        stamp.nsec = map->header.stamp % 1000000000ull;
        map->header.stamp /= 1000ull;
        pcl::toROSMsg(*map, *pointcloud_msg);
        pointcloud_msg->header.frame_id = "map";
        pointcloud_msg->header.stamp = stamp;
        pointcloud_pub.publish(pointcloud_msg);

        // occupancy grid message
        nav_msgs::OccupancyGridPtr occupancy_grid_msg(new nav_msgs::OccupancyGrid());
        occupancy_grid_msg->header.frame_id = "map";
        occupancy_grid_msg->header.seq = 0;
        occupancy_grid_msg->header.stamp = stamp;
        occupancy_grid_msg->info.resolution = occupancy_grid->resolution_;
        occupancy_grid_msg->info.width = occupancy_grid->width_;
        occupancy_grid_msg->info.height = occupancy_grid->height_;
        EigIsometry3d occupancy_grid_origin = occupancy_grid->initial_pose_;
        EigQuaterniond q(occupancy_grid_origin.linear());
        occupancy_grid_msg->info.origin.orientation.x = q.x();
        occupancy_grid_msg->info.origin.orientation.y = q.y();
        occupancy_grid_msg->info.origin.orientation.z = q.z();
        occupancy_grid_msg->info.origin.orientation.w = q.w();
        occupancy_grid_msg->info.origin.position.x = occupancy_grid_origin.translation().x();
        occupancy_grid_msg->info.origin.position.y = occupancy_grid_origin.translation().y();
        occupancy_grid_msg->info.origin.position.z = occupancy_grid_origin.translation().z();
        for(int & i : occupancy_grid->data_) {
            occupancy_grid_msg->data.emplace_back(i);
        }
        occgrid_map_pub.publish(occupancy_grid_msg);

        // marker array initialized
        visualization_msgs::MarkerArray markers;
        markers.markers.resize(1);

        // node markers visualization
        visualization_msgs::Marker& traj_marker = markers.markers[0];
        traj_marker.header.frame_id = "map";
        traj_marker.header.stamp = ros::Time::now();
        traj_marker.ns = "nodes";
        traj_marker.id = 0;
        traj_marker.type = visualization_msgs::Marker::SPHERE_LIST;
        traj_marker.pose.orientation.w = 1.0;
        traj_marker.scale.x = traj_marker.scale.y = traj_marker.scale.z = 0.5;
        traj_marker.points.resize(poses.size());
        traj_marker.colors.resize(poses.size());
        for(int i = 0; i < poses.size(); i++)
        {
            Eigen::Vector3d pos = poses[i].translation();
            traj_marker.points[i].x = pos.x();
            traj_marker.points[i].y = pos.y();
            traj_marker.points[i].z = pos.z();

            double p = static_cast<double>(i) / poses.size();
            traj_marker.colors[i].r = 1.0 - p;
            traj_marker.colors[i].g = p;
            traj_marker.colors[i].b = 0.0;
            traj_marker.colors[i].a = 1.0;
        }
        markers_pub.publish(markers);

        // map->odom transformation
        geometry_msgs::TransformStamped map2odom;
        map2odom.header.frame_id = "map";
        map2odom.child_frame_id = main_reference;
        map2odom.transform.translation.x = poses[poses.size()-1].translation().x();
        map2odom.transform.translation.y = poses[poses.size()-1].translation().y();
        map2odom.transform.translation.z = poses[poses.size()-1].translation().z();
        tf2::Matrix3x3 M_(
                poses[poses.size() - 1].rotation().coeff(0, 0), poses[poses.size() - 1].rotation().coeff(0, 1),
                poses[poses.size() - 1].rotation().coeff(0, 2), poses[poses.size() - 1].rotation().coeff(1, 0),
                poses[poses.size() - 1].rotation().coeff(1, 1), poses[poses.size() - 1].rotation().coeff(1, 2),
                poses[poses.size() - 1].rotation().coeff(2, 0), poses[poses.size() - 1].rotation().coeff(2, 1),
                poses[poses.size() - 1].rotation().coeff(2, 2)
        );
        tf2::Quaternion q_;
        M_.getRotation(q_);
        map2odom.transform.rotation.x = q_.getX();
        map2odom.transform.rotation.y = q_.getY();
        map2odom.transform.rotation.z = q_.getZ();
        map2odom.transform.rotation.w = q_.getW();

        // pose message
        geometry_msgs::PoseStamped pose_;
        pose_.header = map2odom.header;
        pose_.pose.position.x = map2odom.transform.translation.x;
        pose_.pose.position.y = map2odom.transform.translation.y;
        pose_.pose.position.z = map2odom.transform.translation.z;
        pose_.pose.orientation = map2odom.transform.rotation;
        pose_pub.publish(pose_);

        try{
            tf2::Quaternion qq(pose_.pose.orientation.x,
                               pose_.pose.orientation.y,
                               pose_.pose.orientation.z,
                               pose_.pose.orientation.w);


            tf2::Stamped<tf2::Transform> base_to_map(tf2::Transform(qq,tf2::Vector3(pose_.pose.position.x,
                                                                                    pose_.pose.position.y,0.0)).inverse(),
                                                     stamp,
                                                     "base_link");


            geometry_msgs::TransformStamped base_to_map_msg, odom_to_map_msg;
            tf2::convert(base_to_map, base_to_map_msg);
            odom_to_map_msg = tf_buffer.transform(base_to_map_msg, "odom");
            tf2::Stamped<tf2::Transform> odom_to_map;
            tf2::convert(odom_to_map_msg, odom_to_map);

            tf2::convert(odom_to_map.inverse(), latest_transform.transform);
            latest_transform.header.frame_id = "map";
            latest_transform.child_frame_id = "odom";
        }
        catch (tf2::TransformException)
        {
            std::cerr << "Missing tf!" << std::endl;
        }
    }

    /**
     * Roto-Translation matrix into an Odometry Transformation message to be applied.
     *
     * @param stamp Current timestamp of the Roto-Translation Matrix.
     * @param pose Current pose.
     * @param frame_id Frame ID of the considered TF source.
     * @param child_frame_id Child Frame ID of the considered TF dest.
     * @return Odometry transformation
     */
//    geometry_msgs::TransformStamped BridgeVisualizer::matrix2transform(
//            const ros::Time& stamp,
//            const Eigen::Matrix4f& pose,
//            const std::string& frame_id,
//            const std::string& child_frame_id)
//    {
//        // Current pose expressed in a roto-translation matrix converted into quaternion for the rotation component.
//        Eigen::Quaternionf quat(pose.block<3, 3>(0, 0));
//        quat.normalize();
//
//        // odometry output to be prepared
//        geometry_msgs::Quaternion odom_quat;
//
//        // current quaternion rotation
//        odom_quat.w = quat.w();
//        odom_quat.x = quat.x();
//        odom_quat.y = quat.y();
//        odom_quat.z = quat.z();
//
//        // odometry transformation message to be returned
//        geometry_msgs::TransformStamped odom_trans;
//        odom_trans.header.stamp = stamp;
//        odom_trans.header.frame_id = frame_id;
//        odom_trans.child_frame_id = child_frame_id;
//
//        // translation
//        odom_trans.transform.translation.x = pose(0, 3);
//        odom_trans.transform.translation.y = pose(1, 3);
//        odom_trans.transform.translation.z = pose(2, 3);
//
//        // rotation
//        odom_trans.transform.rotation = odom_quat;
//
//        // output transformation message
//        return odom_trans;
//    }
}
