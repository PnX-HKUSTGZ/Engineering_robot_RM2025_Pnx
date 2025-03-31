#include "Filter.hpp"

bool FilterCorner::Update(std::vector<cv::Point2f> & input){
    if(input.size()!=8){
        RCLCPP_ERROR(rclcpp::get_logger("FilterCorner::Update"),"Size of update is [%ld] which is not equal to 8",input.size());
        return 0;
    }
    Eigen::Matrix<double,8,2> In;
    for(int i=0;i<8;i++){
        In(i,0)=input[i].x;
        In(i,1)=input[i].y;
    }
    former=former*(1-k)+In*k;
    for(int i=0;i<8;i++){
        input[i]=cv::Point2f(former(i,0),former(i,1));
    }
    return 1;
}