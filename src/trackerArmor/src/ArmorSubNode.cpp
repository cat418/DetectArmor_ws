#include "ArmorSubNode.h"

//ROS2
#include <tf2_ros/create_timer_ros.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

void ArmorSubNode::CameraToOdom() {
    //创建两个发布器 一个负责从camera_frame->imu_frame 一个负责imu_frame->odom_frame
    auto now = this->get_clock()->now();

    //camera_frame->imu_frame
    geometry_msgs::msg::TransformStamped camera_to_imu;
    camera_to_imu.header.stamp = now;
    camera_to_imu.header.frame_id = "imu_frame";//父坐标系
    camera_to_imu.child_frame_id = "camera_frame";//子坐标系
    camera_to_imu.transform.translation.x = 0.1;//根据实际情况调整
    camera_to_imu.transform.translation.y = 0.0;
    camera_to_imu.transform.translation.z = 0.0;
    camera_to_imu.transform.rotation.x = 0.0;//单位四元数 (无旋转)
    camera_to_imu.transform.rotation.y = 0.0;
    camera_to_imu.transform.rotation.z = 0.0;
    camera_to_imu.transform.rotation.w = 1.0;
    broadcaster_->sendTransform(camera_to_imu);
    RCLCPP_INFO(this->get_logger(), "Broadcasting transform: %s -> %s",
    camera_to_imu.header.frame_id.c_str(),
    camera_to_imu.child_frame_id.c_str());

    //imu_frame->odom_frame
    geometry_msgs::msg::TransformStamped imu_to_odom;
    imu_to_odom.header.stamp = now;
    imu_to_odom.header.frame_id = "odom_frame";//父坐标系
    imu_to_odom.child_frame_id = "imu_frame";//子坐标系
    imu_to_odom.transform.translation.x = 0.0;//根据实际情况调整
    imu_to_odom.transform.translation.y = 0.0;
    imu_to_odom.transform.translation.z = 0.0;
    imu_to_odom.transform.rotation.x = 0.0;//单位四元数 (无旋转)
    imu_to_odom.transform.rotation.y = 0.0;
    imu_to_odom.transform.rotation.z = 0.0;
    imu_to_odom.transform.rotation.w = 1.0;
    broadcaster_->sendTransform(imu_to_odom);
    RCLCPP_INFO(this->get_logger(), "Broadcasting transform: %s -> %s",
    imu_to_odom.header.frame_id.c_str(),
    imu_to_odom.child_frame_id.c_str());
}

//构造函数 初始化这些成员
ArmorSubNode::ArmorSubNode(const rclcpp::NodeOptions &options) : 
    Node("armor_tracker",options),
    m_tf_buffer(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
    m_tf_listener(std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer)),
    m_odom_frame("odom_frame")//初始化参考坐标系
{ 
    //创建发布器
    world_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/armor_world_pose", 10);;
    //创建广播器
    broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    CameraToOdom();
    //创建一个订阅器 用于订阅主题   Qos质量服务策略 将回调函数与订阅器关联  this当前类实例的指针    _1占位符 
    m_armors_sub = this->create_subscription<armor_interfaces::msg::Armors>("/armor", rclcpp::SensorDataQoS(),std::bind(&ArmorSubNode::subArmorsCallback, this, std::placeholders::_1));
}

void ArmorSubNode::subArmorsCallback(const armor_interfaces::msg::Armors::SharedPtr armors_msg) {
    for (auto& armor : armors_msg->armors) {
        geometry_msgs::msg::PoseStamped ps_in_camera;
        ps_in_camera.header = armors_msg->header;
        if( ps_in_camera.header.frame_id.empty() ) {
            ps_in_camera.header.frame_id = "camera_frame";
        }
        ps_in_camera.pose = armor.pose;

        //打印输入位姿
        RCLCPP_INFO(this->get_logger(), "Input Pose: frame_id: %s, position: [%f, %f, %f]",
            ps_in_camera.header.frame_id.c_str(),
            ps_in_camera.pose.position.x,
            ps_in_camera.pose.position.y,
            ps_in_camera.pose.position.z);

        try {
            //使用tf2进行坐标变换
            geometry_msgs::msg::PoseStamped ps_in_odom = m_tf_buffer->transform( ps_in_camera, m_odom_frame );
            //保存变换后的位姿
            armor.world_pose = ps_in_odom.pose;

            //打印变换后的位姿
        RCLCPP_INFO(this->get_logger(), "Transformed Pose: frame_id: %s, position: [%f, %f, %f]",
            m_odom_frame.c_str(),
            ps_in_odom.pose.position.x,
            ps_in_odom.pose.position.y,
            ps_in_odom.pose.position.z);

            // 发布转换后的世界坐标
            geometry_msgs::msg::PoseStamped world_pose_msg;
            //是PoseStamped消息的时间戳字段
            world_pose_msg.header.stamp = this->get_clock()->now();
            //是PoseStamped消息的参考坐标系
            world_pose_msg.header.frame_id = m_odom_frame;
            //消息的核心字段，包含装甲板在世界坐标系odom_frame下的位姿
            world_pose_msg.pose = armor.world_pose;
            world_pose_pub_->publish(world_pose_msg);
        } catch (const tf2::TransformException& e) {
            //记录错误级别的日志信息
            //this(当前节点实例)    get_logger(成员函数，用于获取当前节点的日志对象,日志显示以节点名开头)
            RCLCPP_ERROR(this->get_logger(), "Transform error: %s", e.what());
            return;
        }
    }
}


#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ArmorSubNode)