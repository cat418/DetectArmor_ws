#ifndef ARMOR_SUB_NODE_H
#define ARMOR_SUB_NODE_H

#include <iostream>
//包含rclcpp库中的订阅器接口 用于接收ros2消息
#include <rclcpp/subscription.hpp>
#include <thread>
//c语言的内存管理库 通常用于内存操作
#include <memory.h>

//定义节点及其基本功能
#include <rclcpp/rclcpp.hpp>
#include <image_transport/publisher.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <image_transport/image_transport.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>

#include "armor_interfaces/msg/armor.hpp"

class ArmorSubNode : public rclcpp::Node{
public:
//该构造函数用于初始化节点
    ArmorSubNode(const rclcpp::NodeOptions &options);
    //回调函数 用于处理接收到的订阅消息
    void subArmorsCallback(const armor_interfaces::msg::Armor::SharedPtr armors_msg);
private:
    //ros2的订阅器对象 负责订阅类型为armor_interfaces::msg::Armor的消息
    rclcpp::Subscription<armor_interfaces::msg::Armor>::SharedPtr m_armors_sub;
};

#endif