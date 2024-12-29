// #ifndef PNPSLOVER_H
// #define PNPSLOVER_H

#include "detect.h"

class PnpSlover
{
public:
    PnpSlover() = default;
    ~PnpSlover() = default;
    cv::Mat getMatrix(){ return m_matrix;}
    cv::Mat getDistCoeffs(){ return m_distCoeffs;}
    void calculate_pnp(Armour &a, Light &l);
private:
    cv::Mat m_rvec;//旋转向量
    cv::Mat m_tvec;//平移向量
    cv::Mat m_matrix;//相机内参
    cv::Mat m_distCoeffs;//畸变系数
    std::vector<cv::Point3f> m_small_armor_point3d;//三维坐标
};

// #endif
