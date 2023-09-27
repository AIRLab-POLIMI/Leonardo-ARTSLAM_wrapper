// -*- lots-wrapper-c++ -*-

/* ---------------------------------------------------------------------------------------------------------------------
 * Package: LOTS-WRAPPER
 * Class: Skeleton
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
#include "kernel/skeleton.hpp"

namespace lots::slam::wrapper
{
    using namespace lots::slam::utils;
            void print_header(const std::string& text)
            {
                std::string title(100, '#');
                title.replace((int)(title.length() / 2 - (int)(text.length() / 2)), text.length(), text);
                std::string subtitle(title.length(), '~');
                std::cout << std::endl << subtitle << std::endl << title << std::endl << subtitle << std::endl;
            }

            void Skeleton::registerObserver(SLAMOutputObserver *observer) {
                backend.backend_handler->register_slam_output_observer(observer);
            }

            void Skeleton::registerOdomObserver(OdometryObserver* observer) {
                lidar_list[0].setController(observer);
            }

            /**
             * Initialize kernel modules (Back-End + Loop Detector + Front-End).
             *
             * @param n ROS node
             * @param bridge BridgeVisualizer reference
             * @param config_file Configuration file
             */
            void Skeleton::start(std::shared_ptr<rclcpp::Node> n, const std::string& config_file)
            {
                std::string tmp_prior_odom_topic;
                int lidar_count = parse_num_sensors(config_file, "lidar");
                int camera_count = parse_num_sensors(config_file, "camera");
                int odom_count = parse_num_sensors(config_file, "odom");
                int gnss_count = parse_num_sensors(config_file, "gnss");
                int imu_count = parse_num_sensors(config_file, "imu");
                int radar_count = parse_num_sensors(config_file, "radar");
                int loop_detector_count = parse_num_loop_detectors(config_file);

                // back-end initialization
                print_header(" BACK-END ");
                backend.start(config_file);

                // loop detectors initialization
                print_header(" LOOP-DETECTORS ");
                for (int i = 0; i < loop_detector_count; i++)
                {
                    addLoopDetector(i);
                    loop_detector_list[i].start(config_file);
                }

                /* front-end */
                print_header(" FRONT-END ");

                int sub_disabled = 0;
                auto sensor_qos = rclcpp::QoS(rclcpp::SensorDataQoS());
                for (int i = 0; i < lidar_count; i++)
                {
                    if (parse_sensor_availability(config_file, "lidar", i))
                    {
                        lidar_list.push_back(Lidar(i));
                        tmp_prior_odom_topic = parse_prior_odom_topic(config_file, "lidar", i);
                        if (!tmp_prior_odom_topic.empty()) {
                            lidar_list[i].prior_odom_sub = n->create_subscription<nav_msgs::msg::Odometry>(tmp_prior_odom_topic, 10, std::bind(
                                    &lots::slam::wrapper::Lidar::prior_odom_callback, lidar_list[i], _1));
                        }
                        lidar_list[i].start(&backend, config_file);
                        lidar_list[i].sensor_sub = n->create_subscription<sensor_msgs::msg::PointCloud2>(parse_sensor_topic(config_file, "lidar", i), sensor_qos, std::bind(
                                &lots::slam::wrapper::Lidar::callback,  lidar_list[i], _1));
                        std::string imu_topic;
                        n->get_parameter<std::string>("imu_topic", imu_topic);
                        if(!imu_topic.empty()) {
                            std::cout<<"IMU tight coupling enabled"<<std::endl;
                            lidar_list[i].imu_sub = n->create_subscription<sensor_msgs::msg::Imu>(imu_topic, sensor_qos, std::bind(
                                    &lots::slam::wrapper::Lidar::imu_callback, lidar_list[i], _1));
                        }
                        backend.backend_handler->add_sensor_graph(LIDAR, i);
                    }
                    else
                    {
                        sub_disabled++;
                    }
                }
                lidar_count -= sub_disabled;

                sub_disabled = 0;
                for (int i = 0; i < imu_count; i++)
                {
                    if (parse_sensor_availability(config_file, "imu", i))
                    {
                        imu_list.push_back(Imu(i));
                        imu_list[i].start(&backend, config_file);
                        imu_list[i].sensor_sub = n->create_subscription<sensor_msgs::msg::Imu>(parse_sensor_topic(config_file, "imu", i), sensor_qos, std::bind(
                                &lots::slam::wrapper::Imu::callback,  imu_list[i], _1));
                    }
                    else
                    {
                        sub_disabled++;
                    }
                }
                imu_count -= sub_disabled;

                sub_disabled = 0;
                for (int i = 0; i < gnss_count; i++)
                {
                    if (parse_sensor_availability(config_file, "gnss", i))
                    {
                        gnss_list.push_back(Gnss(i));
                        gnss_list[i].sensor_sub = n->create_subscription<sensor_msgs::msg::NavSatFix>( parse_sensor_topic(config_file, "gnss", i), sensor_qos, std::bind(
                                &lots::slam::wrapper::Gnss::callback, gnss_list[i], _1));
                    }
                    else
                    {
                        sub_disabled++;
                    }
                }
                gnss_count -= sub_disabled;

                /*
                sub_disabled = 0;
                for (int i = 0; i < camera_count; i++)
                {
                    if (parse_sensor_availability(config_file, "camera", i))
                    {
                        addCamera(i, parse_sensor_topic(config_file, "camera", i), parse_sensor_buffer(config_file, "camera", i));
                        camera_list[i].start(n, &backend, config_file);
                        backend.backend_handler->add_sensor_graph("camera", i);
                    }
                    else
                    {
                        sub_disabled++;
                    }
                }
                camera_count -= sub_disabled;
                */

                sub_disabled = 0;
                for (int i = 0; i < radar_count; i++)
                {
                    if (parse_sensor_availability(config_file, "radar", i))
                    {
                        radar_list.push_back(Radar(i));
                        radar_list[i].sensor_sub = n->create_subscription<sensor_msgs::msg::PointCloud2>(parse_sensor_topic(config_file, "radar", i), sensor_qos, std::bind(&lots::slam::wrapper::Radar::callback, radar_list[i], _1));
                        backend.backend_handler->add_sensor_graph(RADAR, i);
                    }
                    else
                    {
                        sub_disabled++;
                    }
                }
                radar_count -= sub_disabled;

                sub_disabled = 0;
                for (int i = 0; i < odom_count; i++)
                {
                    if (parse_sensor_availability(config_file, "odom", i))
                    {
                        odom_list.push_back(Odom(i));
                        odom_list[i].sensor_sub = n->create_subscription<nav_msgs::msg::Odometry>(parse_sensor_topic(config_file, "odom", i), sensor_qos, std::bind(
                            &lots::slam::wrapper::Odom::callback, odom_list[i], _1));
                        backend.backend_handler->add_sensor_graph(ODOMETRY, i);
                    }
                    else
                    {
                        sub_disabled++;
                    }
                }
                odom_count -= sub_disabled;

                // linking back-end with loop detectors
                backend.backend_handler->set_lidar_loop_detector(loop_detector_list[0].loop_detector.get());

                std::string camera_start = (camera_count > 0) ? camera_list[0]._start_color : "";
                std::string lidar_start = (lidar_count > 0) ? lidar_list[0]._start_color : "";
                std::string imu_start = (imu_count > 0) ? imu_list[0]._start_color : "";
                std::string radar_start = (radar_count > 0) ? radar_list[0]._start_color : "";
                std::string odom_start = (odom_count > 0) ? odom_list[0]._start_color : "";
                std::string gnss_start = (gnss_count > 0) ? gnss_list[0]._start_color : "";
                std::string color_end = "\033[0m";

//                bool fancy = false;
//
//                if (fancy) {
//
//                std::cout << "\n\t   \t         +-:%*:=*  -:.---:\n"
//                             "\t \t            *+ -%. *= @:.- \n"
//                             "\t \t            #+ -%:.*= @---  \n"
//                             "\t          ::...::   #+ -+  =-.+--= \n"
//                             "\t          :%@@#:   .*+.              \n"
//                             "\t           =@@-      .::..         ...          ....                      \n"
//                             "\t           =@@:  .+#*===+#%*-  -%@@+=*@%=   :#@@*+++*%#=       -*=::=*= ----.   \n"
//                             "\t           =@@: =@%.      .#@#. *@@   :@@+   =@@      +@@=    :@.    .@ *=     \n"
//                             "\t           -@@::@@.         %@% +@@    @@-   -@@       =@@.   -@:     % **-=  \n"
//                             "\t           -@@:+@@.         =@@.+@%::-#%-    -@@       :%@:    =%-. .+* *=     \n"
//                             "\t           -@@::@@+         +@% +@%::+@%.    -@@       -@@  :----=*=:              \n"
//                             "\t           -@@: -@@+       :@%. +@%   +@%:   =@@      :@@-  =..@= +* .=: =-=.     \n"
//                             "\t           =@@:   =%%+---=*#=  .%@@:   -%@+..*@%++==+##+.      %- +*::%: %:-.       \n"
//                             "\t           =@@=      ::::.     :::::.    .--:.......           @- ++  %: % ::          \n"
//                             "\t           *@@=     =*                                         @- --  -. =--.          \n"
//                             "\t         .=@@@%#*##@@:                                        :=-                        \n"
//                             "\t     :+***++:                                                             -+**+++.       \n"
//                             "\t    .#@+:::+@=                            ...       ....        ..       :%@=:::*@:      \n"
//                             "\t    %@+     =- -@@#+++*@.:*@@=    .*@- =#+==+@. .+#+---=##= .*@@==#@*.  .@@=     *.      \n"
//                             "\t    %@%.        %@-    :  =%*@#.   -% :@#    - .%%       +@# :@%   #@+  .@@#.    .       \n"
//                             "\t    -@@@#=.     #@+---==  -% :%@-  -%  +@@*=.  *@+        #@+:@%  .%%.   =@@@*-.         \n"
//                             "\t     .+%@@@%-   #@+:--+-  :@   +@#::@    :=#@%:*@#        +@+:@%==@@.     .+%@@@#-       \n"
//                             "\t        .=%@@*  %@-     : :@    .#@#@ =     -@* #@=      .@#.:@#  -@%.       :+%@@=      \n"
//                             "\t   =       +@@.:@@#===+%:.*@=     -%@ *#=--=#*.  -##=--=+*=  +@@:  :#@=.+       #@@      \n"
//                             "\t   *-      -@@ ......... ....            ...        ...     .....    ...#.      +@#      \n"
//                             "\t   =@*=:.:=@%:                                                          *@*=:.-+@#.      \n"
//                             "\t    .-:=*+=:                                                             :-:+*+=:        \n"
//                             "\t\t\t\t\t:-+*#%@@@@@@@@@@%#*+-:                        \n"
//                             "\t\t\t\t   :=*%@@@@#*++==----==++*#@@@@%*=:                   \n"
//                             "\t\t\t\t-*@@@%*=:                      *%@@@*-                \n"
//                             "\t\t\t     -#@@@* "+odom_start+"..:::."+color_end+"                  "+radar_start+".:::.."+color_end+" *@@@#-             \n"
//                             "\t\t\t  .+@@@*  "+odom_start+"-*@@@%%@@%+."+color_end+"           "+radar_start+"=#@@%%@@@*-"+color_end+" :*@@@+.          \n"
//                             "\t\t\t +@@@=   "+odom_start+"#@%-     :+@@-"+color_end+"        "+radar_start+"-@@*:     -%@%."+color_end+"  =@@@+         \n"
//                             "\t\t       -@@@=    "+odom_start+"#@#         :@@:"+color_end+" :::: "+radar_start+".@@-         *@%"+color_end+"    =@@@-       \n"
//                             "\t\t      #@@*      "+odom_start+"@@:"+color_end+"   ODOM   "+odom_start+"%@="+color_end+"-@@@@="+radar_start+"-@% "+color_end+"  RADAR  "+radar_start+":@@"+color_end+"      *@@#      \n"
//                             "\t\t    :%@@-       "+odom_start+"%@+         .@@-"+color_end+"      "+radar_start+":@@.         +@@"+color_end+"       -@@%:    \n"
//                             "\t\t   :@@%.        "+odom_start+":@@*.      -%@+"+color_end+"        "+radar_start+"+@@-      .*@@:"+color_end+"        .%@@:   \n"
//                             "\t\t  .@@%.           "+odom_start+"+@@%#**#@@@@:"+color_end+"         "+radar_start+"#@@@#***%@@+."+color_end+"          .%@@.  \n"
//                             "\t\t  %@@.           +*:"+odom_start+".-===*@@@@*"+color_end+"        "+radar_start+":@@@@@===-:"+color_end+":**           .@@%  \n"
//                             "\t\t *@@-           #@#       "+odom_start+"-++-"+color_end+"          "+radar_start+":**+."+color_end+"      #@%.          -@@* \n"
//                             "\t\t.@@#      "+gnss_start+":-==-:-="+color_end+"           ("<< odom_count <<")      ("<< radar_count <<")           =-:"+imu_start+"-==-:"+color_end+"      #@@.\n"
//                             "\t\t+@@-   "+gnss_start+"=#@@#**#@@#-"+color_end+"                                "+imu_start+"-#@@%#*#@@%="+color_end+"   -@@+\n"
//                             "\t\t@@@   "+gnss_start+"#@#:      -%@*"+color_end+"                              "+imu_start+"*@%=      :#@%."+color_end+"  @@#\n"
//                             "\t\t@@%  "+gnss_start+"*@*          %@%@#-"+color_end+"                      "+imu_start+":#%%@@          *@#"+color_end+"  %@@\n"
//                             "\t\t@@%  "+gnss_start+"%@:"+color_end+"   GNSS   "+gnss_start+"+@@@@@."+color_end+"("<< gnss_count <<")   LOTS-SLAM   ("<< imu_count <<")"+imu_start+"@@@@@*"+color_end+"    IMU   "+imu_start+"-@%"+color_end+"  #@@\n"
//                             "\t\t@@%  "+gnss_start+"*@#          %@##*:"+color_end+"                      "+imu_start+":*%#@@.         #@*"+color_end+"  %@@\n"
//                             "\t\t@@@   "+gnss_start+"#@%-     .=%@+"+color_end+"                              "+imu_start+"+@@+.     -%@#"+color_end+"   @@#\n"
//                             "\t\t+@@-   "+gnss_start+"-*@@%##%@%*:"+color_end+"                                "+imu_start+".+%@@%%%@@*-"+color_end+"   -@@+\n"
//                             "\t\t.@@#      "+gnss_start+".::::"+color_end+":-+           ("<< camera_count <<")      ("<< lidar_count <<")           +="+imu_start+":::::."+color_end+"      #@@.\n"
//                             "\t\t *@@-           *@%.      "+camera_start+":+*=."+color_end+"         "+lidar_start+"=**="+color_end+"      .%@*           -@@* \n"
//                             "\t\t  %@@.           ==:-"+camera_start+"+****@@@@@"+color_end+"        "+lidar_start+"*@@@@%***+-"+color_end+":==           .@@%  \n"
//                             "\t\t  .@@%.          "+camera_start+".#@@#+==*%@@@="+color_end+"        "+lidar_start+".@@@%*==+*@@#:"+color_end+"          .%@@.  \n"
//                             "\t\t   :@@%.        "+camera_start+"-@@=       :#@#"+color_end+"        "+lidar_start+"*@%:       =@@-"+color_end+"        .%@@:   \n"
//                             "\t\t    :%@@-       "+camera_start+"@@=          @@-"+color_end+"      "+lidar_start+"-@@.         =@@"+color_end+"       -@@%:    \n"
//                             "\t\t      #@@*      "+camera_start+"@@-"+color_end+"  CAMERA  "+camera_start+"%@="+color_end+"-@@@@="+lidar_start+"=@@"+color_end+"   LIDAR  "+lidar_start+":@@"+color_end+"      *@@#      \n"
//                             "\t\t       -@@@=    "+camera_start+"*@%.        =@@."+color_end+"      "+lidar_start+".@@=         #@#"+color_end+"    =@@@-       \n"
//                             "\t\t\t +@@@=   "+camera_start+"*@@+:   .=#@%:"+color_end+"        "+lidar_start+".%@#=.   :+@@*"+color_end+"   =@@@+         \n"
//                             "\t\t\t  .+@@@*:  "+camera_start+"+#@@@@@%*-"+color_end+"            "+lidar_start+"-*%@@@@@#+"+color_end+"  :*@@@+.          \n"
//                             "\t\t\t     -#@@@*-.                            .-*@@@#-             \n"
//                             "\t\t\t\t-*@@@%*=:                    :=*%@@@*-                \n"
//                             "\t\t\t\t   :=*%@@@@#*++=------=++*#@@@@%*=:                   \n"
//                             "\t\t\t\t\t:-=*##%@@@@@@@@%##*=-:  " << std::endl;
//                } else {
                    std::cout <<camera_start<<"camera"<<color_end<<std::endl;
                    std::cout <<lidar_start<<"lidar"<<color_end<<std::endl;
                    std::cout <<imu_start<<"imu"<<color_end<<std::endl;
                    std::cout <<radar_start<<"radar"<<color_end<<std::endl;
                    std::cout <<odom_start<<"odom"<<color_end<<std::endl;
                    std::cout <<gnss_start<<"gnss"<<color_end<<std::endl;
//                }
            }
        }
