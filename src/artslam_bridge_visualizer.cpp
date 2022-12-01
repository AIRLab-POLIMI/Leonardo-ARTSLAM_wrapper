// -*- artslam-c++ -*-

/* ---------------------------------------------------------------------------------------------------------------------
 * Package: ARTSLAM-WRAPPER
 * Class: ARTSLAMBridgeVisualizer
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

// Header library
#include "artslam_bridge_visualizer.h"

// ROS Messages libraries
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>

// TFs libraries
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

// PCL libraries
#include <pcl_conversions/pcl_conversions.h>


namespace artslam::laser3d
{
    /**
     * Initialize the bridge-visualizer for ARTSLAM-ROS functionalities.
     */
    ARTSLAMBridgeVisualizer::ARTSLAMBridgeVisualizer() : tf_listener(tf_buffer)
    {
        // Dispatcher initialization for the visualizer
        dispatcher = std::make_unique<core::utils::Dispatcher>(MODULE_ID, 1);

        // ROS Publishers initialization
        markers_pub = handler.advertise<visualization_msgs::MarkerArray>(MARKER_TOPIC, 16);
        pointcloud_pub = handler.advertise<sensor_msgs::PointCloud2>(POINTCLOUD_TOPIC, 1);
        pose_pub = handler.advertise<geometry_msgs::PoseStamped>(POSE_TOPIC, 10);

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

    /**
     * Initialize the bridge-visualizer for ARTSLAM-ROS functionalities.
     *
     * @param nh ROS node handler address already existent.
     */
    ARTSLAMBridgeVisualizer::ARTSLAMBridgeVisualizer(ros::NodeHandle &nh) : tf_listener(tf_buffer)
    {
        // Setting the ROS handler
        set_handler(nh);

        // Dispatcher initialization for the visualizer
        dispatcher = std::make_unique<core::utils::Dispatcher>(MODULE_ID, 1);

        // ROS Publishers initialization
        markers_pub = handler.advertise<visualization_msgs::MarkerArray>(MARKER_TOPIC, 16);
        pointcloud_pub = handler.advertise<sensor_msgs::PointCloud2>(POINTCLOUD_TOPIC, 1);
        pose_pub = handler.advertise<geometry_msgs::PoseStamped>(POSE_TOPIC, 10);

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

    /**
     * It signals that a new filtered point cloud has been received and it is going to be displayed.
     *
     * @param pointcloud Filtered PCL pointcloud to be displayed.
     */
    void ARTSLAMBridgeVisualizer::update_filtered_pointcloud_observer(pcl::PointCloud<Point3I>::ConstPtr pointcloud)
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
    void ARTSLAMBridgeVisualizer::update_slam_output_observer(
        pcl::PointCloud<Point3I>::Ptr map,
        std::vector<Eigen::Isometry3d> poses)
    {
        // sending map and poses to the visualizer through the dispatcher
        std::cout << "update_slam_output_observer" << std::endl;
        dispatcher->dispatch([this, map, poses]{draw_map_and_poses(map, poses);});
    }

    /**
     * It draws a new received pointcloud.
     *
     * @param pointcloud Filtered PCL pointcloud to be displayed.
     */
    void ARTSLAMBridgeVisualizer::draw_pointcloud(const pcl::PointCloud<Point3I>::ConstPtr& pointcloud)
    {
        // pointcloud ros message
        sensor_msgs::PointCloud2Ptr pointcloud_msg(new sensor_msgs::PointCloud2());

        // pointcloud pcl
        pcl::PointCloud<Point3I>::Ptr pointcloud_pcl(new pcl::PointCloud<Point3I>);

        // point cloud publishing
        *pointcloud_pcl = *pointcloud;
        pointcloud_pcl->header.stamp /= 1000ull;
        pcl::toROSMsg(*pointcloud_pcl, *pointcloud_msg);
        pointcloud_msg->header.frame_id = "base_link";
        pointcloud_pub.publish(pointcloud_msg);
    }

    /**
     * Signals that map and poses are ready to be displayed.
     *
     * @param map Map to be displayed.
     * @param poses Poses to be displayed.
     */
    void ARTSLAMBridgeVisualizer::draw_map_and_poses(pcl::PointCloud<Point3I>::Ptr map,
                                                     std::vector<EigIsometry3d> poses)
    {
        // pointcloud message
        sensor_msgs::PointCloud2Ptr pointcloud_msg(new sensor_msgs::PointCloud2());
        map->header.stamp /= 1000ull;
        pcl::toROSMsg(*map, *pointcloud_msg);
        pointcloud_msg->header.frame_id = "map";
        pointcloud_pub.publish(pointcloud_msg);

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
        map2odom.child_frame_id = "base_link";
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
        geometry_msgs::PoseStamped pose_, pose_t;
        pose_.header = map2odom.header;
        pose_.pose.position.x = map2odom.transform.translation.x;
        pose_.pose.position.y = map2odom.transform.translation.y;
        pose_.pose.position.z = map2odom.transform.translation.z;
        pose_.pose.orientation = map2odom.transform.rotation;
        pose_pub.publish(pose_);

        try{
            // adjust TF pose from odom to map reference frame
            geometry_msgs::PoseStamped odom2map;
            tf2::Transform latest_tf_;

            // orientation of the last marker
            tf2::Quaternion q(pose_.pose.orientation.x, pose_.pose.orientation.y,
                              pose_.pose.orientation.z, pose_.pose.orientation.w);

            if (true) // TODO: remove it if you want 3D
            {
                double roll, pitch, yaw;
                tf2::Matrix3x3 M;
                M.setRotation(q);
                M.getRPY(roll, pitch, yaw);
                q.setRPY(0, 0, yaw);
                pose_.pose.position.z = 0;
            }

            // temporary tf
            tf2::Transform tmp_tf(q, tf2::Vector3(pose_.pose.position.x, pose_.pose.position.y, pose_.pose.position.z));

            // first phase
            geometry_msgs::PoseStamped tmp_tf_stamped;
            tmp_tf_stamped.header.frame_id = "base_link";
            tmp_tf_stamped.header.stamp = pose_.header.stamp;
            tf2::toMsg(tmp_tf.inverse(), tmp_tf_stamped.pose);
            tf_buffer.transform(tmp_tf_stamped, odom2map, "odom");
            tf2::convert(odom2map.pose, latest_tf_);

            // second phase
            geometry_msgs::TransformStamped tf_stamped;
            tf_stamped.header.stamp = pose_.header.stamp;
            tf_stamped.header.frame_id = "map";
            tf_stamped.child_frame_id = "odom";
            tf2::convert(latest_tf_.inverse(), tf_stamped.transform);
            tf_broadcaster.sendTransform(tf_stamped);
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
    geometry_msgs::TransformStamped ARTSLAMBridgeVisualizer::matrix2transform(
        const ros::Time& stamp,
        const Eigen::Matrix4f& pose,
        const std::string& frame_id,
        const std::string& child_frame_id)
    {
        // Current pose expressed in a roto-translation matrix converted into quaternion for the rotation component.
        Eigen::Quaternionf quat(pose.block<3, 3>(0, 0));
        quat.normalize();

        // odometry output to be prepared
        geometry_msgs::Quaternion odom_quat;

        // current quaternion rotation
        odom_quat.w = quat.w();
        odom_quat.x = quat.x();
        odom_quat.y = quat.y();
        odom_quat.z = quat.z();

        // odometry transformation message to be returned
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = stamp;
        odom_trans.header.frame_id = frame_id;
        odom_trans.child_frame_id = child_frame_id;

        // translation
        odom_trans.transform.translation.x = pose(0, 3);
        odom_trans.transform.translation.y = pose(1, 3);
        odom_trans.transform.translation.z = pose(2, 3);

        // rotation
        odom_trans.transform.rotation = odom_quat;

        // output transformation message
        return odom_trans;
    }
}