#include "template_ros_2_package/simple_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[])
{
    // Inicializa o ROS 2
    rclcpp::init(argc, argv);
    // Cria o nó e o mantém em execução
    rclcpp::spin(std::make_shared<template_ros_2_package::SimpleNode>());
    // Finaliza o ROS 2
    rclcpp::shutdown();
    return 0;
}