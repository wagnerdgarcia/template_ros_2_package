#include "template_ros_2_package/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor;
    auto lifecycle_node = std::make_shared<template_ros_2_package::LifecycleNode>(rclcpp::NodeOptions());
    executor.add_node(lifecycle_node->get_node_base_interface());
    executor.spin();
    rclcpp::shutdown();
    return 0;
}