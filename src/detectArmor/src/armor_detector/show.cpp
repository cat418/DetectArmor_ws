#include "detect.h"
#include "pnpslover.h"
#include "MvCameraControl.h"
#include "HikDriver/HikDriver.h"
#include <cv_bridge/cv_bridge.h>

//表示创建一个 名为"armor_detector"的ros2节点类 options参数允许传递额外的节点选项
Armor_detector::Armor_detector(const rclcpp::NodeOptions &options) :
    Node("armor_detector",options)
{
    //创建装甲板消息发布器
    //rclcpp::SensorDataQoS() 表示使用传感器数据的质量服务（QoS）。该QoS设置有助于确保图像数据等高频传输的数据在网络拥堵时也能保证合适的传输性能。
    m_armors_publish = this->create_publisher<armor_interfaces::msg::Armors>("/armor",rclcpp::SensorDataQoS());
    
        // HikDriver hik_driver(0);
        // if (hik_driver.isConnected()) {
        //     //设置自动增益 曝光
        //     hik_driver.setAutoGain(1);
        //     hik_driver.setAutoExposureTime(1);
        //     // hik_driver.setExposureTime(4000);
        //     // hik_driver.setGain(15);
        //     hik_driver.showParamInfo();
        //     hik_driver.startReadThread();
        // }
        //创建Armor Light
        Armour a;
        Light l;
        m_cap.open("/home/xzq/Downloads/4.mp4");
        if( !m_cap.isOpened() ) {
            std::cout << "视频打开失败" << std::endl;
            return ;
        }
        while (rclcpp::ok()) {
        l.light_rect.clear();
        a.two_Light.clear();
        //HikFrame Frame = hik_driver.getFrame();
        //cv::Mat img = Frame.getRgbFrame()->clone();
        cv::Mat img = read_frame();
        if (img.empty()) {
            continue;
        }
        //预处理
        cv::Mat frame = findLight_deal_frame(img,limit);
        //如果处理后的图像为空，直接显示图像
        if (frame.empty()) {
            std::cout<<"frame是空的"<<std::endl;
            cv::imshow("armor1",img);
            //将图像转化为ROS2类型 创建cv_bridge对象并填充数据
            auto bridge = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img);
        }else{
        //检测轮廓
        std::vector<std::vector<cv::Point>> contours = check_contours(frame);
        //检测灯条 
        detect_light( contours, l);
        //匹配灯条
        match_light( frame, l, a);
        //匹配装甲板
        match_armour( l, a);
        //绘制
        draw_armor( img, l, a);
        cv::imshow("armor",img);
        //将图像转化为ROS2类型 创建cv_bridge对象并填充数据
        auto bridge = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img);
        //创建Armors消息
        armor_interfaces::msg::Armors armors_mag;
        //解pnp
        PnpSlover pnp;
        pnp.calculate_pnp( a, l, armors_mag);
        a.all_four_point.clear();
        //发布消息
        m_armors_publish->publish( armors_mag );
        int fps = m_cap.get( cv::CAP_PROP_FPS );
        if( fps != 0 ) {
            cv::waitKey( 1000 / fps );
        }else{
            cv::waitKey( 30 );
        }
        }
    }
}

//这行代码将 ArmorPubNode 类注册为一个可加载的 ROS 2 组件
// 在ROS 2 系统中动态加载和卸载。
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(Armor_detector);
