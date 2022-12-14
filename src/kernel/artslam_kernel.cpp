// Header library

#include "kernel/artslam_kernel.h"

namespace artslam
{
    namespace laser3d
    {
        void ARTSLAMKernel::start(ros::NodeHandle* mt_nh, ARTSLAMBridgeVisualizer* bridge, std::string config_file)
        {
            // back-end initialization
            backend.start(bridge, config_file);

            // loop detectors initialization
            loop_detectors.start(config_file);

            // front-end initialization
            lidar.start(mt_nh, &backend, config_file);
            imu.start(mt_nh, &backend, config_file);
            gnss.start(mt_nh, &backend, config_file);

            // linking back-end with loop detectors
            backend.backend_handler->set_loop_detector(loop_detectors.loop_detector.get());
        }
    }
}
