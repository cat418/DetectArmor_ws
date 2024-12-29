#include "detect.h"
#include <vector>

bool comparePoints(const cv::Point2f& p1, const cv::Point2f& p2) {
    return (p1.x+p1.y)<(p2.x+p2.y);
}

inline float distance( cv::Point2f point1, cv::Point2f point2 ) {
    return sqrt(pow((point1.x-point1.x),2)+pow((point2.y-point2.y),2));
}

//绘制
void Armor_detector::draw_armor(cv::Mat &frame,Light &l,Armour &a) {
    //根据x+y的值排序 可以判断出八个点的位置
    for( auto & r : a.two_Light) {
        cv::Point2f point_rect[4];
        r.first.points(point_rect);
        
        cv::Point2f point_rect2[4];
        r.second.points(point_rect2);

        std::vector<cv::Point2f> points(point_rect,point_rect+4 );
        std::vector<cv::Point2f> points2(point_rect2,point_rect2+4 );

        std::sort(points.begin(),points.end(),comparePoints);
        std::sort(points2.begin(),points2.end(),comparePoints);

        if(points[0].x<points2[1].x) {
            std::vector<cv::Point> contour;
            contour.push_back( points[0] );
            contour.push_back( points2[1] );
            contour.push_back( points2[3] );
            contour.push_back( points[2] );
            float tan = std::max(abs( points[0].y - points2[1].y ) / abs( points[0].x - points2[1].x ) ,
            abs( points[0].y - points[2].y ) / abs( points[0].x - points[2].x ));
            if( cv::contourArea(contour) > 500 && tan > 1 ) {
                cv::line(frame,points[0],points2[1],cv::Scalar(0,255,0),2,8,0);
                cv::line(frame,points2[1],points2[3],cv::Scalar(0,255,0),2,8,0);
                cv::line(frame,points2[3],points[2],cv::Scalar(0,255,0),2,8,0);
                cv::line(frame,points[2],points[0],cv::Scalar(0,255,0),2,8,0);
                //特征点:左上 左下 右下 右上
                a.four_point.push_back( points[0] );
                a.four_point.push_back( points[2] );
                a.four_point.push_back( points2[3] );
                a.four_point.push_back( points2[1] );
                // for ( int i=0; i<a.four_point.size();i++) {
                //     cv::circle(frame,a.four_point[i],5,cv::Scalar(0,255,0),-1);
                //     std::string num = std::to_string(i);
                //     cv::putText(frame, num, a.four_point[i] + cv::Point2f(10,-10),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(255,0,0),1);
                // }
            }
        }else {
            std::vector<cv::Point> contour;
            contour.push_back( points[0] );
            contour.push_back( points2[1] );
            contour.push_back( points2[3] );
            contour.push_back( points[2] );
            float tan = std::max(abs( points[1].y - points2[0].y ) / abs( points[1].x - points2[0].x ) ,
            abs( points2[2].y - points2[0].y ) / abs( points2[2].x - points2[0].x ));
            if( cv::contourArea(contour) > 500 && tan > 1 ) {
                cv::line(frame,points2[0],points[1],cv::Scalar(0,255,0),2,8,0);
                cv::line(frame,points[1],points[3],cv::Scalar(0,255,0),2,8,0);
                cv::line(frame,points[3],points2[2],cv::Scalar(0,255,0),2,8,0);
                cv::line(frame,points2[2],points2[0],cv::Scalar(0,255,0),2,8,0);
                //特征点:左上 左下 右下 右上
                a.four_point.push_back( points2[0]);
                a.four_point.push_back( points2[2] );
                a.four_point.push_back( points[3] );
                a.four_point.push_back( points[1] );
                // for ( int i=0; i<a.four_point.size();i++) {
                //     cv::circle(frame,a.four_point[i],5,cv::Scalar(0,255,0),-1);
                //     std::string num = std::to_string(i);
                //     cv::putText(frame, num, a.four_point[i] + cv::Point2f(10,-10),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(255,0,0),1);
                // }
            }
        }
    }
}