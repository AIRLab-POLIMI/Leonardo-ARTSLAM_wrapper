#include "controller/controller.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "artslam_wrapper_node");
    artslam::lots::wrapper::Controller c;

    ros::spin();

    return EXIT_SUCCESS;
}