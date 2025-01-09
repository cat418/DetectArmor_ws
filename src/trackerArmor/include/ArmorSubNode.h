#ifndef ARMOR_SUB_NODE_H
#define ARMOR_SUB_NODE_H

#include <iostream>
#include <Eigen/Dense>
#include <thread>
//包含rclcpp库中的订阅器接口 用于接收ros2消息
#include <rclcpp/subscription.hpp>
#include <thread>
#include <string>
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
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "armor_interfaces/msg/armor.hpp"
#include "armor_interfaces/msg/armors.hpp"

class ArmorSubNode : public rclcpp::Node{
public:
//该构造函数用于初始化节点
    ArmorSubNode(const rclcpp::NodeOptions &options);
    //定义发布器
    void CameraToOdom(); 
    //回调函数 用于处理接收到的订阅消息
    void subArmorsCallback(const armor_interfaces::msg::Armors::SharedPtr armors_msg);
private:
   
    //ros2的订阅器对象 负责订阅类型为armor_interfaces::msg::Armor的消息
    rclcpp::Subscription<armor_interfaces::msg::Armors>::SharedPtr m_armors_sub;
    //发布器
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr world_pose_pub_;
    //管理和存储TF坐标变换 提供一个缓冲区 存储从不同坐标系之间的转换数据
    std::shared_ptr<tf2_ros::Buffer> m_tf_buffer;
    //负责订阅tf消息
    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener;
    //用于tf变换的参考坐标系名称
    std::string m_odom_frame;
    //广播坐标变换
    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
    //表示定时器的基本接口
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif