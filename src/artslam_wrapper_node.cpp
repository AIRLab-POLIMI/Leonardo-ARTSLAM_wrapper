#include "controller/controller.hpp"
#include <pcl/pcl_config.h>

int main(int argc, char **argv)
{
    std::cout << PCL_VERSION_PRETTY << std::endl;

    rclcpp::init(argc, argv);
    lots::slam::wrapper::Controller c;


    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(c.get_node());
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
