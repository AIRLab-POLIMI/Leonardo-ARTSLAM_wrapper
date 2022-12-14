// Header library

#include "kernel/artslam_front_end.h"

namespace artslam
{
    namespace laser3d
    {
        void ARTSLAMFrontEnd::start(std::string config_file, bool _prefilterer, bool _tracker, bool _ground_detector)
        {
            /* prefilterer */
            if (_prefilterer)
            {
                Prefilterer::Configuration prefilterer_configuration =
                        artslam::laser3d::parse_prefilterer_configuration(config_file);
                prefilterer = std::make_shared<Prefilterer>(prefilterer_configuration);
            }
            else
            {
                prefilterer = nullptr;
            }

            /* tracker */
            if (_tracker)
            {
                Registration::Configuration registration_tracker_configuration =
                        parse_registration_tracker_configuration(config_file);
                Registration registration(registration_tracker_configuration);
                Tracker::Configuration tracker_configuration = parse_tracker_configuration(config_file);
                tracker = std::make_shared<Tracker>(tracker_configuration, registration.registration_method());
            }
            else
            {
                tracker = nullptr;
            }

            /* ground detector */
            if (_ground_detector)
            {
                GroundDetector::Configuration ground_detector_configuration =
                        parse_ground_detector_configuration(config_file);
                ground_detector = std::make_shared<GroundDetector>(ground_detector_configuration);
            }
            else
            {
                ground_detector = nullptr;
            }


            /* SLAM pipeline construction */
            if (_prefilterer)
            {
                if (_tracker)
                {
                    prefilterer->register_filtered_pointcloud_observer(tracker.get());
                }

                if (_ground_detector)
                {
                    prefilterer->register_filtered_pointcloud_observer(ground_detector.get());
                }
            }
        }
    }
}
