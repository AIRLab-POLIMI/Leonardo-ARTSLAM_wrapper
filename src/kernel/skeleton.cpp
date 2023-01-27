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
#include "kernel/skeleton.h"

namespace artslam::lots::wrapper
{
            void print_header(std::string text)
            {
                std::string title(100, '#');
                title.replace((int)(title.length() / 2 - (int)(text.length() / 2)), text.length(), text);
                std::string subtitle(title.length(), '~');
                std::cout << std::endl << subtitle << std::endl << title << std::endl << subtitle << std::endl;
            }

            /**
             * Initialize kernel modules (Back-End + Loop Detector + Front-End).
             *
             * @param mt_nh ROS node handler
             * @param bridge BridgeVisualizer reference
             * @param config_file Configuration file
             */
            void Skeleton::start(ros::NodeHandle* mt_nh, BridgeVisualizer* bridge, std::string config_file)
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
                backend.start(bridge, config_file);

                // loop detectors initialization
                print_header(" LOOP-DETECTORS ");
                for (int i = 0; i < loop_detector_count; i++)
                {
                    addLoopDetector(i, parse_name_loop_detectors(config_file, i));
                    loop_detector_list[i].start(config_file);
                }

                /* front-end */
                print_header(" FRONT-END ");

                int sub_disabled = 0;
                for (int i = 0; i < lidar_count; i++)
                {
                    if (parse_sensor_availability(config_file, "lidar", i))
                    {
                        addLidar(i, parse_sensor_topic(config_file, "lidar", i), parse_sensor_buffer(config_file, "lidar", i));
                        tmp_prior_odom_topic = parse_prior_odom_topic(config_file, "lidar", i);
                        if (tmp_prior_odom_topic != "/0") {
                            lidar_list[i].set_prior_odom_topic(tmp_prior_odom_topic);
                        }
                        lidar_list[i].start(mt_nh, &backend, config_file);
                        backend.backend_handler->add_sensor_graph("lidar", i);
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
                        addImu(i, parse_sensor_topic(config_file, "imu", i), parse_sensor_buffer(config_file, "imu", i));
                        imu_list[i].start(mt_nh, &backend, config_file);
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
                        addGnss(i, parse_sensor_topic(config_file, "gnss", i), parse_sensor_buffer(config_file, "gnss", i));
                        gnss_list[i].start(mt_nh, &backend, config_file);
                    }
                    else
                    {
                        sub_disabled++;
                    }
                }
                gnss_count -= sub_disabled;

                sub_disabled = 0;
                for (int i = 0; i < camera_count; i++)
                {
                    if (parse_sensor_availability(config_file, "camera", i))
                    {
                        addCamera(i, parse_sensor_topic(config_file, "camera", i), parse_sensor_buffer(config_file, "camera", i));
                        camera_list[i].start(mt_nh, &backend, config_file);
                        backend.backend_handler->add_sensor_graph("camera", i);
                    }
                    else
                    {
                        sub_disabled++;
                    }
                }
                camera_count -= sub_disabled;

                sub_disabled = 0;
                for (int i = 0; i < radar_count; i++)
                {
                    if (parse_sensor_availability(config_file, "radar", i))
                    {
                        addRadar(i, parse_sensor_topic(config_file, "radar", i), parse_sensor_buffer(config_file, "radar", i));
                        radar_list[i].start(mt_nh, &backend, config_file);
                        backend.backend_handler->add_sensor_graph("radar", i);
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
                        addOdom(i, parse_sensor_topic(config_file, "odom", i), parse_sensor_buffer(config_file, "odom", i));
                        odom_list[i].start(mt_nh, &backend, config_file);
                        backend.backend_handler->add_sensor_graph("odom", i);
                    }
                    else
                    {
                        sub_disabled++;
                    }
                }
                odom_count -= sub_disabled;

                // linking back-end with loop detectors
                if (lidar_count > 0)
                    backend.backend_handler->set_loop_detector(loop_detector_list[0].loop_detector.get());

                std::string camera_start = (camera_count > 0) ? camera_list[0]._start_color : "";
                std::string lidar_start = (lidar_count > 0) ? lidar_list[0]._start_color : "";
                std::string imu_start = (imu_count > 0) ? imu_list[0]._start_color : "";
                std::string radar_start = (radar_count > 0) ? radar_list[0]._start_color : "";
                std::string odom_start = (odom_count > 0) ? odom_list[0]._start_color : "";
                std::string gnss_start = (gnss_count > 0) ? gnss_list[0]._start_color : "";
                std::string color_end = "\033[0m";

                std::cout << "\n\t   \t         +-:%*:=*  -:.---:\n"
                             "\t \t            *+ -%. *= @:.- \n"
                             "\t \t            #+ -%:.*= @---  \n"
                             "\t          ::...::   #+ -+  =-.+--= \n"
                             "\t          :%@@#:   .*+.              \n"
                             "\t           =@@-      .::..         ...          ....                      \n"
                             "\t           =@@:  .+#*===+#%*-  -%@@+=*@%=   :#@@*+++*%#=       -*=::=*= ----.   \n"
                             "\t           =@@: =@%.      .#@#. *@@   :@@+   =@@      +@@=    :@.    .@ *=     \n"
                             "\t           -@@::@@.         %@% +@@    @@-   -@@       =@@.   -@:     % **-=  \n"
                             "\t           -@@:+@@.         =@@.+@%::-#%-    -@@       :%@:    =%-. .+* *=     \n"
                             "\t           -@@::@@+         +@% +@%::+@%.    -@@       -@@  :----=*=:              \n"
                             "\t           -@@: -@@+       :@%. +@%   +@%:   =@@      :@@-  =..@= +* .=: =-=.     \n"
                             "\t           =@@:   =%%+---=*#=  .%@@:   -%@+..*@%++==+##+.      %- +*::%: %:-.       \n"
                             "\t           =@@=      ::::.     :::::.    .--:.......           @- ++  %: % ::          \n"
                             "\t           *@@=     =*                                         @- --  -. =--.          \n"
                             "\t         .=@@@%#*##@@:                                        :=-                        \n"
                             "\t     :+***++:                                                             -+**+++.       \n"
                             "\t    .#@+:::+@=                            ...       ....        ..       :%@=:::*@:      \n"
                             "\t    %@+     =- -@@#+++*@.:*@@=    .*@- =#+==+@. .+#+---=##= .*@@==#@*.  .@@=     *.      \n"
                             "\t    %@%.        %@-    :  =%*@#.   -% :@#    - .%%       +@# :@%   #@+  .@@#.    .       \n"
                             "\t    -@@@#=.     #@+---==  -% :%@-  -%  +@@*=.  *@+        #@+:@%  .%%.   =@@@*-.         \n"
                             "\t     .+%@@@%-   #@+:--+-  :@   +@#::@    :=#@%:*@#        +@+:@%==@@.     .+%@@@#-       \n"
                             "\t        .=%@@*  %@-     : :@    .#@#@ =     -@* #@=      .@#.:@#  -@%.       :+%@@=      \n"
                             "\t   =       +@@.:@@#===+%:.*@=     -%@ *#=--=#*.  -##=--=+*=  +@@:  :#@=.+       #@@      \n"
                             "\t   *-      -@@ ......... ....            ...        ...     .....    ...#.      +@#      \n"
                             "\t   =@*=:.:=@%:                                                          *@*=:.-+@#.      \n"
                             "\t    .-:=*+=:                                                             :-:+*+=:        \n"
                             "\t\t\t\t\t:-+*#%@@@@@@@@@@%#*+-:                        \n"
                             "\t\t\t\t   :=*%@@@@#*++==----==++*#@@@@%*=:                   \n"
                             "\t\t\t\t-*@@@%*=:                      *%@@@*-                \n"
                             "\t\t\t     -#@@@* "+odom_start+"..:::."+color_end+"                  "+radar_start+".:::.."+color_end+" *@@@#-             \n"
                             "\t\t\t  .+@@@*  "+odom_start+"-*@@@%%@@%+."+color_end+"           "+radar_start+"=#@@%%@@@*-"+color_end+" :*@@@+.          \n"
                             "\t\t\t +@@@=   "+odom_start+"#@%-     :+@@-"+color_end+"        "+radar_start+"-@@*:     -%@%."+color_end+"  =@@@+         \n"
                             "\t\t       -@@@=    "+odom_start+"#@#         :@@:"+color_end+" :::: "+radar_start+".@@-         *@%"+color_end+"    =@@@-       \n"
                             "\t\t      #@@*      "+odom_start+"@@:"+color_end+"   ODOM   "+odom_start+"%@="+color_end+"-@@@@="+radar_start+"-@% "+color_end+"  RADAR  "+radar_start+":@@"+color_end+"      *@@#      \n"
                             "\t\t    :%@@-       "+odom_start+"%@+         .@@-"+color_end+"      "+radar_start+":@@.         +@@"+color_end+"       -@@%:    \n"
                             "\t\t   :@@%.        "+odom_start+":@@*.      -%@+"+color_end+"        "+radar_start+"+@@-      .*@@:"+color_end+"        .%@@:   \n"
                             "\t\t  .@@%.           "+odom_start+"+@@%#**#@@@@:"+color_end+"         "+radar_start+"#@@@#***%@@+."+color_end+"          .%@@.  \n"
                             "\t\t  %@@.           +*:"+odom_start+".-===*@@@@*"+color_end+"        "+radar_start+":@@@@@===-:"+color_end+":**           .@@%  \n"
                             "\t\t *@@-           #@#       "+odom_start+"-++-"+color_end+"          "+radar_start+":**+."+color_end+"      #@%.          -@@* \n"
                             "\t\t.@@#      "+gnss_start+":-==-:-="+color_end+"           ("<< odom_count <<")      ("<< radar_count <<")           =-:"+imu_start+"-==-:"+color_end+"      #@@.\n"
                             "\t\t+@@-   "+gnss_start+"=#@@#**#@@#-"+color_end+"                                "+imu_start+"-#@@%#*#@@%="+color_end+"   -@@+\n"
                             "\t\t@@@   "+gnss_start+"#@#:      -%@*"+color_end+"                              "+imu_start+"*@%=      :#@%."+color_end+"  @@#\n"
                             "\t\t@@%  "+gnss_start+"*@*          %@%@#-"+color_end+"                      "+imu_start+":#%%@@          *@#"+color_end+"  %@@\n"
                             "\t\t@@%  "+gnss_start+"%@:"+color_end+"   GNSS   "+gnss_start+"+@@@@@."+color_end+"("<< gnss_count <<")   LOTS-SLAM   ("<< imu_count <<")"+imu_start+"@@@@@*"+color_end+"    IMU   "+imu_start+"-@%"+color_end+"  #@@\n"
                             "\t\t@@%  "+gnss_start+"*@#          %@##*:"+color_end+"                      "+imu_start+":*%#@@.         #@*"+color_end+"  %@@\n"
                             "\t\t@@@   "+gnss_start+"#@%-     .=%@+"+color_end+"                              "+imu_start+"+@@+.     -%@#"+color_end+"   @@#\n"
                             "\t\t+@@-   "+gnss_start+"-*@@%##%@%*:"+color_end+"                                "+imu_start+".+%@@%%%@@*-"+color_end+"   -@@+\n"
                             "\t\t.@@#      "+gnss_start+".::::"+color_end+":-+           ("<< camera_count <<")      ("<< lidar_count <<")           +="+imu_start+":::::."+color_end+"      #@@.\n"
                             "\t\t *@@-           *@%.      "+camera_start+":+*=."+color_end+"         "+lidar_start+"=**="+color_end+"      .%@*           -@@* \n"
                             "\t\t  %@@.           ==:-"+camera_start+"+****@@@@@"+color_end+"        "+lidar_start+"*@@@@%***+-"+color_end+":==           .@@%  \n"
                             "\t\t  .@@%.          "+camera_start+".#@@#+==*%@@@="+color_end+"        "+lidar_start+".@@@%*==+*@@#:"+color_end+"          .%@@.  \n"
                             "\t\t   :@@%.        "+camera_start+"-@@=       :#@#"+color_end+"        "+lidar_start+"*@%:       =@@-"+color_end+"        .%@@:   \n"
                             "\t\t    :%@@-       "+camera_start+"@@=          @@-"+color_end+"      "+lidar_start+"-@@.         =@@"+color_end+"       -@@%:    \n"
                             "\t\t      #@@*      "+camera_start+"@@-"+color_end+"  CAMERA  "+camera_start+"%@="+color_end+"-@@@@="+lidar_start+"=@@"+color_end+"   LIDAR  "+lidar_start+":@@"+color_end+"      *@@#      \n"
                             "\t\t       -@@@=    "+camera_start+"*@%.        =@@."+color_end+"      "+lidar_start+".@@=         #@#"+color_end+"    =@@@-       \n"
                             "\t\t\t +@@@=   "+camera_start+"*@@+:   .=#@%:"+color_end+"        "+lidar_start+".%@#=.   :+@@*"+color_end+"   =@@@+         \n"
                             "\t\t\t  .+@@@*:  "+camera_start+"+#@@@@@%*-"+color_end+"            "+lidar_start+"-*%@@@@@#+"+color_end+"  :*@@@+.          \n"
                             "\t\t\t     -#@@@*-.                            .-*@@@#-             \n"
                             "\t\t\t\t-*@@@%*=:                    :=*%@@@*-                \n"
                             "\t\t\t\t   :=*%@@@@#*++=------=++*#@@@@%*=:                   \n"
                             "\t\t\t\t\t:-=*##%@@@@@@@@%##*=-:  " << std::endl;
            }
        }
