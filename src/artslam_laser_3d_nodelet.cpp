#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <prefilterer.h>
#include <iostream>

#include <filesystem>
#include <execution>
#include <artslam_io/pointcloud_io.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include "registration.h"
#include "tracker.h"
#include "ground_detector.h"
#include "graph_handler.h"
#include "information_matrix_calculator.h"
#include "loop_detector.h"
#include "visualizer_bridge.h"
#include <backend_handler.h>
#include <configuration_parser.cpp>
#include <backend_handler.cpp>
#include <artslam_laser_3d_wrapper/OfflineSLAM.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>

namespace artslam::laser3d {
    class ArtslamLaserWrapper : public nodelet::Nodelet {
    public:
        ArtslamLaserWrapper() : value_(0) {}

    private:
//        void get_filepaths(const std::string &path, const std::string &extension, std::vector<std::string> &filepaths) {
//            for (const auto &p: std::filesystem::directory_iterator(path)) {
//                if (p.is_regular_file()) {
//                    if (p.path().extension().string() == extension) {
//                        filepaths.emplace_back(p.path());
//                    }
//                }
//            }
//
//            std::sort(std::execution::par_unseq, filepaths.begin(), filepaths.end());
//        }

        virtual void onInit() {
            private_nh = getPrivateNodeHandle();
            mt_nh = getMTNodeHandle();
            private_nh.getParam("value", value_);
            pub = private_nh.advertise<std_msgs::Float64>("out", 10);
            map_pub = mt_nh.advertise<sensor_msgs::PointCloud2>("/hdl_graph_slam/map_points", 1, true);
            cloud_sub_ = mt_nh.subscribe("/velodyne_points", 64, &ArtslamLaserWrapper::cloud_callback, this);
            imu_sub_ = mt_nh.subscribe("/imu_data", 1024, &ArtslamLaserWrapper::imu_callback, this);
            gnss_sub_ = mt_nh.subscribe("/gnss_data", 1024, &ArtslamLaserWrapper::gnss_callback, this);
            service_ = private_nh.advertiseService("OfflineSLAM", &ArtslamLaserWrapper::offline_slam, this);

            private_nh.getParam("configuration_file", config_file_);
            private_nh.getParam("results_path", results_path_);

            _outfile.open(results_path_+"/timestamps.txt", std::ios::out);

            // prefilterer
            Prefilterer::Configuration prefilterer_configuration = artslam::laser3d::parse_prefilterer_configuration(config_file_);
            prefilterer_ = std::make_shared<Prefilterer>(prefilterer_configuration);

            // tracker
            Registration::Configuration registration_tracker_configuration = parse_registration_tracker_configuration(config_file_);
            Registration registration(registration_tracker_configuration);
            Tracker::Configuration tracker_configuration = parse_tracker_configuration(config_file_);
            tracker_ = std::make_shared<Tracker>(tracker_configuration, registration.registration_method());

            // ground detector
            GroundDetector::Configuration ground_detector_configuration = parse_ground_detector_configuration(config_file_);
            ground_detector_ = std::make_shared<GroundDetector>(ground_detector_configuration);

            // graph handler
            graph_handler_ = std::make_shared<GraphHandler>();

            // information matrix calculator
            InformationMatrixCalculator::Configuration information_matrix_calculator_configuration = parse_information_matrix_calculator(config_file_);
            information_matrix_calculator_ = std::make_shared<InformationMatrixCalculator>(information_matrix_calculator_configuration);

            // loop detector
            Registration::Configuration loop_detector_registration_configuration = parse_registration_loop_detector_configuration(config_file_);
            Registration loop_detector_registration(loop_detector_registration_configuration);
            LoopDetector::Configuration loop_detector_configuration = parse_loop_detector_configuration(config_file_);
            loop_detector_ = std::make_shared<LoopDetector>(loop_detector_configuration, loop_detector_registration.registration_method());

            // backend handler
            BackendHandler::Configuration backend_handler_configuration = parse_backend_handler_configuration(config_file_);
            backend_handler_ = std::make_shared<BackendHandler>(backend_handler_configuration);
            backend_handler_->set_graph_handler(graph_handler_.get());
            backend_handler_->set_information_matrix_calculator(information_matrix_calculator_.get());
            backend_handler_->set_loop_detector(loop_detector_.get());

            // visualization wrapper
            visualization_wrapper_.set_handle(mt_nh);

            // SLAM pipeline construction
            prefilterer_->register_filtered_pointcloud_observer(tracker_.get());
            prefilterer_->register_filtered_pointcloud_observer(ground_detector_.get());
            tracker_->register_keyframe_observer(backend_handler_.get());
            ground_detector_->register_floor_coefficients_observer(backend_handler_.get());
            backend_handler_->register_slam_output_observer(&visualization_wrapper_);
        }

        bool offline_slam(artslam_laser_3d_wrapper::OfflineSLAM::Request &req,
                          artslam_laser_3d_wrapper::OfflineSLAM::Request &res) {
            backend_handler_->save_results(results_path_);
            return true;
//            std::vector<std::string> slam_paths = parse_slam_paths(config_file_);
//
//            std::vector<std::string> pointclouds_filepaths;
//            get_filepaths(slam_paths[0], ".bin", pointclouds_filepaths);
//
//            std::vector<std::string> oxts_filepaths;
//            get_filepaths(slam_paths[2], ".txt", oxts_filepaths);
//
//            artslam::core::io::KITTI_Reader kitti_reader({false, boost::log::trivial::trace});
//            std::vector<uint64_t> timestamps, oxts_timestamps;
//            kitti_reader.read_timestamps(slam_paths[1], timestamps);
//            kitti_reader.read_timestamps(slam_paths[3], oxts_timestamps);
//
//            // load immediately all IMU and GNSS data
//            for (int i = 0; i < oxts_filepaths.size(); i++) {
//                IMU3D_MSG::Ptr imu3d_msg = kitti_reader.read_imu(oxts_filepaths[i]);
//                imu3d_msg->header_.frame_id_ = "";
//                imu3d_msg->header_.sequence_ = i;
//                imu3d_msg->header_.timestamp_ = oxts_timestamps[i];
//                //prefilterer.update_raw_imu_observer(imu3d_msg);
//                //backend_handler.update_raw_imu_observer(imu3d_msg);
//
//                GeoPointStamped_MSG::Ptr gnss_msg = kitti_reader.read_gps(oxts_filepaths[i]);
//                gnss_msg->header_.frame_id_ = "";
//                gnss_msg->header_.sequence_ = i;
//                gnss_msg->header_.timestamp_ = oxts_timestamps[i];
//                //backend_handler.update_raw_gnss_observer(gnss_msg);
//            }
//
//            for (int i = 0; i < 200; i++) {
//                pcl::PointCloud<Point3I>::Ptr pointcloud = kitti_reader.read_pointcloud(pointclouds_filepaths[i]);
//                pointcloud->header.frame_id = "";
//                pointcloud->header.seq = i;
//                pointcloud->header.stamp = timestamps[i];
//
//                prefilterer_->update_raw_pointcloud_observer(pointcloud);
//                usleep(100000);
//            }
//
//            sleep(5);
//
//            backend_handler_->save_results(results_path_);
//
//            std::cout << "[END] Size: " << pointclouds_filepaths.size() << std::endl;
//            std::cout << "[END] Prefilterer: " << prefilterer_->total_time_ << " - " << prefilterer_->count_ << std::endl;
//            std::cout << "[END] Tracker: " << tracker_->total_time_ << " - " << tracker_->count_ << std::endl;
//            std::cout << "[END] GroundDetector: " << ground_detector_->total_time_ << " - " << ground_detector_->count_ << std::endl;
//            std::cout << "[END] LoopDetector: " << loop_detector_->total_time_ << " - " << loop_detector_->count_ << std::endl;
//            std::cout << "[END] GraphHandler: " << graph_handler_->total_time_ << " - " << graph_handler_->count_ << std::endl;
//            std::cout << "[END] BackendHandler: " << backend_handler_->total_time_ << " - " << backend_handler_->count_ << std::endl;
//
//            return true;
        }

        void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
            if(!ros::ok()) {
                return;
            }

            pcl::PointCloud<Point3I>::Ptr cloud(new pcl::PointCloud<Point3I>());
            pcl::fromROSMsg(*cloud_msg, *cloud);
            cloud->header.seq = count_;
            count_++;

            cloud->header.stamp = cloud_msg->header.stamp.toNSec();
            prefilterer_->update_raw_pointcloud_observer(cloud);

            _outfile<<cloud->header.stamp<<" "<<count_<<std::endl;
            count_++;
        }

        void imu_callback(const sensor_msgs::ImuPtr& imu_msg) {
            const auto& imu_orientation = imu_msg->orientation;
            const auto& imu_acceleration = imu_msg->linear_acceleration;

            geometry_msgs::Vector3Stamped acc_imu;
            geometry_msgs::Vector3Stamped acc_base;
            geometry_msgs::QuaternionStamped quat_imu;
            geometry_msgs::QuaternionStamped quat_base;

            quat_imu.header.frame_id = acc_imu.header.frame_id = imu_msg->header.frame_id;
            quat_imu.header.stamp = acc_imu.header.stamp = ros::Time(0);
            acc_imu.vector = imu_msg->linear_acceleration;
            quat_imu.quaternion = imu_msg->orientation;

            try {
                tf_listener_.transformVector("base_link", acc_imu, acc_base);
                tf_listener_.transformQuaternion("base_link", quat_imu, quat_base);
            }
            catch (tf::TransformException &ex) {
                    std::cerr << ex.what() << std::endl;
                    return;
            }

            IMU3D_MSG::Ptr conv_imu_msg(new IMU3D_MSG);
            conv_imu_msg->header_.timestamp_ = imu_msg->header.stamp.toNSec();
            conv_imu_msg->header_.frame_id_ = "base_link";
            tf::Stamped<tf::Quaternion> quat;
            tf::quaternionStampedMsgToTF(quat_base, quat);
            tf::quaternionTFToEigen(quat, conv_imu_msg->orientation_);
            tf::Stamped<tf::Vector3> acc;
            tf::vector3StampedMsgToTF(acc_base, acc);
            tf::vectorTFToEigen(acc, conv_imu_msg->linear_acceleration_);

            //prefilterer_->update_raw_imu_observer(conv_imu_msg);
            backend_handler_->update_raw_imu_observer(conv_imu_msg);
        }

        void gnss_callback(const sensor_msgs::NavSatFixConstPtr& navsat_msg) {
			if(gnss_count_ == 0) {
					tf::StampedTransform transform;
					try {
						tf_listener_.lookupTransform("base_link", navsat_msg->header.frame_id, ros::Time::now(), transform);
					} catch (std::exception &e) {
						std::cerr << "failed to find gps transform!!" << std::endl;
						return;
					}
					EigVector3d g2l_translation(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
					backend_handler_->set_gps_to_lidar_translation(g2l_translation);
				}
				GeoPointStamped_MSG::Ptr conv_gnss_msg(new GeoPointStamped_MSG);
				conv_gnss_msg->header_.timestamp_ = navsat_msg->header.stamp.toNSec();
				conv_gnss_msg->header_.frame_id_ = "base_link";
				conv_gnss_msg->header_.sequence_ = gnss_count_;
				gnss_count_++;
				conv_gnss_msg->latitude_ = navsat_msg->latitude;
				conv_gnss_msg->longitude_ = navsat_msg->longitude;
				conv_gnss_msg->altitude_ = navsat_msg->altitude;
				conv_gnss_msg->covariance_type_ = navsat_msg->position_covariance_type;
				for(int i = 0; i < 9; i++){
					if(conv_gnss_msg->covariance_type_ == 0) {
						   conv_gnss_msg->covariance_[i] = 0.0;
					} else {
						conv_gnss_msg->covariance_[i] = navsat_msg->position_covariance[i];
					}
				}
				backend_handler_->update_raw_gnss_observer(conv_gnss_msg);
        }

        int gnss_count_ = 0;
        ros::NodeHandle mt_nh;
        ros::NodeHandle private_nh;
        ros::Publisher pub, map_pub;
        ros::Subscriber cloud_sub_;
        ros::Subscriber imu_sub_;
        ros::Subscriber gnss_sub_;
        tf::TransformListener tf_listener_;
        double value_;
        int count_ = 0;
        ros::ServiceServer service_;
        std::ofstream _outfile;

        // SLAM building blocks
        std::shared_ptr<Prefilterer> prefilterer_;
        std::shared_ptr<Tracker> tracker_;
        std::shared_ptr<GroundDetector> ground_detector_;
        std::shared_ptr<GraphHandler> graph_handler_;
        std::shared_ptr<LoopDetector> loop_detector_;
        std::shared_ptr<InformationMatrixCalculator> information_matrix_calculator_;
        std::shared_ptr<BackendHandler> backend_handler_;

        // general params
        std::string config_file_;
        std::string results_path_;

        // nodelet elements
        VisualizerBridge visualization_wrapper_;
    };

    PLUGINLIB_EXPORT_CLASS(artslam::laser3d::ArtslamLaserWrapper, nodelet::Nodelet)
}
