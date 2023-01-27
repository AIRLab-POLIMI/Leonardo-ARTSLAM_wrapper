#include "controller/controller.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "artslam_wrapper_node");
    artslam::lots::wrapper::Controller c;

    ros::MultiThreadedSpinner spinner(2); // Use 4 threads
    spinner.spin(); // spin() will not return until the node has been shutdown

    return EXIT_SUCCESS;
}