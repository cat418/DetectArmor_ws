#include "ArmorSubNode.h"
ArmorSubNode::ArmorSubNode(const rclcpp::NodeOptions &options) : 
    Node("armor_tracker",options)
{
    //创建一个订阅器 用于订阅主题   Qos质量服务策略 将回调函数与订阅器关联  this当前类实例的指针    _1占位符 
    m_armors_sub = this->create_subscription<armor_interfaces::msg::Armor>("/armor", rclcpp::SensorDataQoS(),std::bind(&ArmorSubNode::subArmorsCallback, this, std::placeholders::_1));
}

void ArmorSubNode::subArmorsCallback(const armor_interfaces::msg::Armor::SharedPtr armors_msg) {
    for ( int i = 0; i < 4; i ++ ) {
        std::cout << "特征点" <<i+1<<":" << armors_msg->apexs[i].x << "," << armors_msg->apexs[i].y << "," << armors_msg->apexs[i].z << std::endl;
    }
    std::cout << "pose :" << "("<<armors_msg->pose.position.x << "," << armors_msg->pose.position.y << "," << armors_msg->pose.position.z << ")" << std::endl;
    std::cout << "      " << "("<<armors_msg->pose.orientation.x << "," << armors_msg->pose.orientation.y << "," << armors_msg->pose.orientation.z << "," << armors_msg->pose.orientation.w << ")" << std::endl;

    std::cout << "颜色:" <<armors_msg->color<< std::endl;
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ArmorSubNode)