#include "ros2-multithreading/multithread_pub_sub.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto pub_node =  std::make_shared<multithread::PubNode>();
    auto sub2_node = std::make_shared<multithread::Sub2Node>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(pub_node->get_node_base_interface());
    executor.add_node(sub2_node->get_node_base_interface());
    executor.spin();
    rclcpp::shutdown();

    return EXIT_SUCCESS;
}


