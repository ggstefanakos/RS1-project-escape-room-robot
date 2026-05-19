#include "myagv_odometry/myAGV.h"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <iostream>
#include <cstring> 

double linearX = 0.0;
double linearY = 0.0;
double angularZ = 0.0;

//using namespace std;

void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    linearX = msg->linear.x;
    linearY = msg->linear.y;
    angularZ = msg->angular.z;
    //RCLCPP_INFO(rclcpp::get_logger("myagv_odometry_node"), "cmdCallback: linearX: %.2f, linearY: %.2f, angularZ: %.2f", linearX, linearY, angularZ);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyAGV>("myagv_odometry_node");

    if (!node->init()) {
        RCLCPP_ERROR(node->get_logger(), "myAGV initialized failed!");
        return 1;
    }
    RCLCPP_INFO(node->get_logger(), "myAGV initialized successful!");

    auto sub = node->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 50, cmdCallback);
    rclcpp::Rate loop_rate(100);

    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        node->execute(linearX, linearY, angularZ);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}