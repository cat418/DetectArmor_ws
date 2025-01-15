#ifndef _DETECT_H_
#define _DETECT_H_


#include <iostream>
#include <opencv2/opencv.hpp>
#include "HikDriver/HikDriver.h"


#include <thread>
#include <memory.h>//c语言的内存操作头文件 通常用于直接操作内存

#include <rclcpp/rclcpp.hpp>
//用于ros2中传输图像消息
#include <image_transport/publisher.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <image_transport/image_transport.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>

//自定义消息类型
#include "armor_interfaces/msg/armor.hpp"
#include "armor_interfaces/msg/armors.hpp"

//类声明
class Light;

class Armour
{
/**
 *@brief: 装甲板的构造函数
 *@param: 装甲板的数据 
 */
public:
    Armour() = default;
    ~Armour() = default;

    std::vector<std::pair< cv::RotatedRect, cv::RotatedRect>> two_Light;
    std::vector<std::vector<cv::Point2f>> all_four_point;//二维坐标
    std::vector<float> orientation;//四元数
    cv::Point3f point_3D_center;

    float get_width(){ return m_width; };
    float get_height(){ return m_height; };
private:
    float m_width = 13.04;
    float m_height = 12;
};

class Light
{
/**
 * @brief:灯条的构造函数
 * @param:灯条的数据
 */
public:
    Light() = default;
    ~Light() = default;
   
    //获取灯条的旋转矩形
    std::vector<cv::RotatedRect> light_rect;
    float get_height(){ return m_height; };
private:
    float m_height = 5.5;
};

class Armor_detector : public rclcpp::Node
{
/**
 * @brief:装甲板识别流程
 */
public:
    //构造函数
    Armor_detector(const rclcpp::NodeOptions &options);
    //析构函数
    ~ Armor_detector() = default;
    //读取图像
    cv::Mat read_frame();
    //预处理
    cv::Mat findLight_deal_frame(cv::Mat frame,int limit);
    //检测轮廓
    std::vector<std::vector<cv::Point>> check_contours(cv::Mat preprocessing_frame);
    //检测灯条
    void detect_light(std::vector<std::vector<cv::Point>> contours,Light &l);
    //匹配灯条
    void match_light( cv::Mat frame, Light &l, Armour &a);
    //匹配装甲板 
    void match_armour(Light &l,Armour &a);
    //绘制
    void draw_armor(cv::Mat &frame,Light &l,Armour &a);
    //获取二值化阈值
    int get_limit(){ return limit; };
private:
    cv::VideoCapture m_cap;
    //发布消息
    rclcpp::Publisher<armor_interfaces::msg::Armors>::SharedPtr m_armors_publish;
    //发布图像
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_image_publish;
    //线程对象
    std::thread m_push_core;
    std::thread m_detect_core;
    //二值化阈值
    int limit = 150;
};

#endif