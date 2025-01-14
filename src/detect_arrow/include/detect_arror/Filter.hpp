#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>

#ifndef __Filter__
#define __Filter__

class FilterCorner{
    public:
    FilterCorner(int k):k(k){
        former<<0,0,
        0,0,
        0,0,
        0,0,
        0,0,
        0,0,
        0,0,
        0,0;
        RCLCPP_INFO(rclcpp::get_logger("FilterCorner"),"FilterCorner Creat");
    };
    bool Update(std::vector<cv::Point2f> & input);
    private:
    double k;
    Eigen::Matrix<double,8,2> former;
};

#endif