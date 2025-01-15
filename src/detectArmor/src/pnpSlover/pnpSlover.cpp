#include "pnpslover.h"
#include <opencv2/core.hpp>

//世界坐标系和相机坐标系 z向前 ,x向右 ,y向下
//设置相机内参矩阵 畸变系数
void PnpSlover::calculate_pnp(Armour &a, Light &l,   armor_interfaces::msg::Armors& armors) {
    m_matrix = (cv::Mat_<double>(3,3) <<
    1809.870863937357, 0, 690.2243899941344,
    0, 1812.98436546181, 562.730599006955,
    0, 0, 1
    );
    m_distCoeffs = (cv::Mat_<double>(1,5) <<-0.06356538390336125, 0.03903999928160801, -0.001395636011335281, -0.001940042987071293, 0.2630557748237058);
    
    // m_matrix = (cv::Mat_<double>(3,3) <<
    // 1814.304470709766, 0, 680.0866838997997,
    // 0, 1816.46254639905, 572.3860870746728,
    // 0, 0, 1
    // );
    // m_distCoeffs = (cv::Mat_<double>(1,5) <<-0.1140280192150111, 0.6440347079155343, 0.0002982650615947499, -0.002297783671074355, -1.876992654445899);

    float small_armro_half_x = a.get_width() / 2.0 / 100.0 ,  small_light_half_y = l.get_height() / 2.0 / 100.0;
    m_small_armor_point3d.push_back(cv::Point3f(-0, small_armro_half_x, small_light_half_y));   //左上
    m_small_armor_point3d.push_back(cv::Point3f(-0,  small_armro_half_x, -small_light_half_y));   //左下
    m_small_armor_point3d.push_back(cv::Point3f(-0,  small_armro_half_x,  small_light_half_y));   //右下
    m_small_armor_point3d.push_back(cv::Point3f(-0, -small_armro_half_x, -small_light_half_y));   //右上
    //装甲板中心3D坐标
    cv::Point3f center(0, 0, 0);
    // 计算四个点的几何中心（即平均坐标）
    for (const auto& point : m_small_armor_point3d) {
        center += point;
    }
    // 求平均值
    center.x /= 4.0;
    center.y /= 4.0;
    center.z /= 4.0;
   
    for ( auto & a_four_point : a.all_four_point ) {
    
    cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);
    armor_interfaces::msg::Armor armor_msg;

    if( m_small_armor_point3d.size() == a_four_point.size() && a_four_point.size() == 4 ) {
        armor_msg.pose.position.x = center.x;
        armor_msg.pose.position.y = center.y;
        armor_msg.pose.position.z = center.z;

        cv::solvePnP( m_small_armor_point3d, a_four_point, m_matrix, m_distCoeffs, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
        float distance = cv::norm(tvec);
        armor_msg.distance_to_center = distance;

        //求四元数
        cv::Mat rvec_rotation_matrix;
        //OpenCV函数 将旋转向量转换为旋转矩阵
        cv::Rodrigues(rvec, rvec_rotation_matrix);
        //将旋转矩阵转化为tf2格式
        tf2::Matrix3x3 tf2_rotation_matrix(
            rvec_rotation_matrix.at<double>(0, 0),rvec_rotation_matrix.at<double>(0, 1),rvec_rotation_matrix.at<double>(0, 2),
            rvec_rotation_matrix.at<double>(1, 0),rvec_rotation_matrix.at<double>(1, 1),rvec_rotation_matrix.at<double>(1, 2),
            rvec_rotation_matrix.at<double>(2, 0),rvec_rotation_matrix.at<double>(2, 1),rvec_rotation_matrix.at<double>(2, 2)
        );
        //从旋转矩阵提取四元数
        tf2::Quaternion tf2_q;
        tf2_rotation_matrix.getRotation( tf2_q );
        //转换为ROS消息格式的四元数
        armor_msg.pose.orientation = tf2::toMsg(tf2_q);

        //四个特征点
        for ( int i = 0; i < 4; i ++ ) {
            armor_msg.apexs[i].x= a_four_point[i].x;
            armor_msg.apexs[i].y=a_four_point[i].y;
            armor_msg.apexs[i].z=0.0f;
        }
        armors.armors.push_back( armor_msg );
      }
    } 
}