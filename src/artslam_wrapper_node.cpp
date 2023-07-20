#include "controller/controller.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    lots::slam::wrapper::Controller c;


    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(c.get_node());
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
