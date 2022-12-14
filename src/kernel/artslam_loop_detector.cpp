// Header library

#include "kernel/artslam_loop_detector.h"

namespace artslam
{
    namespace laser3d
    {
        void ARTSLAMLoopDetector::start(std::string config_file)
        {
            /* loop detector */
            Registration::Configuration loop_detector_registration_configuration =
                    parse_registration_loop_detector_configuration(config_file);
            Registration loop_detector_registration(loop_detector_registration_configuration);
            LoopDetector::Configuration loop_detector_configuration =
                    parse_loop_detector_configuration(config_file);
            loop_detector = std::make_shared<LoopDetector>(loop_detector_configuration,
                                                           loop_detector_registration.registration_method());
        }
    }
}
