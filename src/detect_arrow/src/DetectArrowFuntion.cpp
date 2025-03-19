#include "DetectArrow.hpp"

template<typename T>
cv::Point_<T> LocalCornerOpitimize(const cv::Mat & BinaryImage ,cv::Point_<T> Corner, int MaskRadius, int blockSize, int ksize, double k){

    CV_Assert(BinaryImage.type()==CV_8U);

    cv::Point_<T> OpitmizedCorner;
    double dis=1e9;
    //(col,row) (x,y)
    cv::Point StartPoint(std::max(0,int(Corner.x-MaskRadius)),std::max(0,int(Corner.y-MaskRadius)));
    int maxrow=BinaryImage.rows,maxcol=BinaryImage.cols;

    int row=-StartPoint.y+int(Corner.y)+1 + std::min(int(maxrow-Corner.y),MaskRadius);
    int col=-StartPoint.x+int(Corner.x)+1 + std::min(int(maxcol-Corner.x),MaskRadius);
    #ifdef test_LocalCornerOpitimize

    RCLCPP_INFO(rclcpp::get_logger("LocalCornerOpitimize"),"size: %ld %ld",row,col);
    RCLCPP_INFO(rclcpp::get_logger("LocalCornerOpitimize"),"maxsize: %ld %ld",maxrow,maxcol);
    RCLCPP_INFO(rclcpp::get_logger("LocalCornerOpitimize"),"StartPoint: %ld %ld",StartPoint.x,StartPoint.y);
    RCLCPP_INFO(rclcpp::get_logger("LocalCornerOpitimize"),"Corner: %ld %ld",int(Corner.x),int(Corner.y));
    RCLCPP_INFO(rclcpp::get_logger("LocalCornerOpitimize"),"??: %ld %ld",std::min(int(maxrow-Corner.y),MaskRadius),std::min(int(maxcol-Corner.x),MaskRadius));
    #endif

    cv::Mat Masked(cv::Size(row,col),CV_8U),HattisResult;

    for(int i=0;i<row;i++){
        for(int e=0;e<col;e++){
            Masked.at<uint8_t>(i,e)=BinaryImage.at<uint8_t>(i+StartPoint.y,e+StartPoint.x);
        }
    }

    RCLCPP_INFO(rclcpp::get_logger("LocalCornerOpitimize"),"1");
    int BlockSize=std::min(blockSize,std::min(row,col));
    cv::cornerHarris(Masked,HattisResult,blockSize,ksize,k);
    RCLCPP_INFO(rclcpp::get_logger("LocalCornerOpitimize"),"1");

    std::vector<cv::Point> ResultPoints;


    for(int i=0;i<row;i++){
        for(int e=0;e<col;e++){
            RCLCPP_INFO(rclcpp::get_logger("LocalCornerOpitimize"),"pos(x,y) [%ld,%ld] %f",e,i,HattisResult.at<float>(i,e));
        }
    }


    for(int i=0;i<row;i++){
        for(int e=0;e<col;e++){
            if(HattisResult.at<float>(i,e) >= k){
                ResultPoints.push_back(cv::Point(e,i)+StartPoint);
            }
        }
    }

    if(!ResultPoints.size()){
        return Corner;
    }
    for(auto & i : ResultPoints){
        if(DistancePoints(i,Corner)<=dis){
            dis=DistancePoints(i,Corner);
            OpitmizedCorner=cv::Point_<T>(i.x,i.y);
        }
    }

#ifdef test_LocalCornerOpitimize

    for(int i=0;i<row;i++){
        for(int e=0;e<col;e++){
            RCLCPP_INFO(rclcpp::get_logger("LocalCornerOpitimize"),"pos(x,y) [%ld,%ld] %f",e,i,HattisResult.at<float>(i,e));
        }
    }

    cv::Mat coloredImage(BinaryImage.size(),CV_64FC3);
    for(int i=0;i<BinaryImage.size().height;i++){
        for(int e=0;e<BinaryImage.size().width;e++){
            coloredImage.at<cv::Vec3d>(i,e)[0]=BinaryImage.at<uint8_t>(i,e);
        }
    }
    coloredImage.at<cv::Vec3d>(int(Corner.y),int(Corner.x))=cv::Vec3d(225,225,225);
    coloredImage.at<cv::Vec3d>(int(OpitmizedCorner.y),int(OpitmizedCorner.x))=cv::Vec3d(0,225,225);

    cv::imshow("coloredImage",coloredImage);
    cv::waitKey(0);

#endif

    return OpitmizedCorner;
}

template cv::Point_<double> LocalCornerOpitimize<double>(const cv::Mat & BinaryImage ,cv::Point_<double> Corner, int MaskRadius, int blockSize, int ksize, double k);
