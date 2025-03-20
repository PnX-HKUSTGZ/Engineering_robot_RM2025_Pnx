#include "DetectArrow.hpp"

cv::Mat Arrow_detector::Adapted_PreProgress(const cv::Mat & OriginalImage){
    std::vector<cv::Mat> SplitImage;
    cv::split(OriginalImage,SplitImage);

    #ifdef DetectorRectangle_test
    cv::imshow("Adapted_PreProgress OriginalImage",OriginalImage);
    #endif

    cv::Mat GreyImage(SplitImage[0].size(),SplitImage[0].type()),BinaryImage,DilatedImage;

    cv::mixChannels(std::vector<cv::Mat>{SplitImage[0],SplitImage[2]},
        std::vector<cv::Mat>{GreyImage},
        std::vector<int>{
            0,0,
            1,0
    });
    
    cv::Mat GaussGrayImage;
    cv::GaussianBlur(GreyImage,
        GaussGrayImage,
        cv::Size(5,5),
        1.5
    );

    cv::adaptiveThreshold(GaussGrayImage
        ,BinaryImage
        ,DetectRectangleMaxValue
        ,cv::ADAPTIVE_THRESH_GAUSSIAN_C,
        cv::THRESH_BINARY,
        DetectRectangleBlockSize,
        DetectRectangleC);

    #ifdef DetectorRectangle_test
    cv::imshow("Adapted_PreProgress BinaryImage",BinaryImage);
    #endif

    return BinaryImage;

}


std::vector<cv::Point2f> Arrow_detector::TargetRectangle(const cv::Mat & BinaryImage,cv::Mat & Image){
    Counters Cornerscounters;
    std::vector<cv::RotatedRect> CounterRotatedRects;
    std::vector<int> counter_size;
    cv::findContours(BinaryImage,Cornerscounters,cv::RETR_LIST,cv::CHAIN_APPROX_SIMPLE);
    Counter result;
    cv::RotatedRect rect;
    cv::Size2f siz;
}

bool Arrow_detector::MainDetectArrow_Rectangle(const cv::Mat & OriginalImage){
    OriginalImage.copyTo(OriginalImage_Rectangle);
    cv::Mat BinaryImage=Adapted_PreProgress(OriginalImage_Rectangle);
    
    std::vector<cv::Point2f> TargetRectangleResult=TargetRectangle(BinaryImage,OriginalImage_Rectangle);



}