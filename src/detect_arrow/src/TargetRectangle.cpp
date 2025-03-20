#include "DetectArrow.hpp"

bool Arrow_detector::MainDetectArrow_Rectangle(const cv::Mat & OriginalImage){
    OriginalImage.copyTo(OriginalImage_Rectangle);
    cv::Mat BinaryImage=Adapted_PreProgress(OriginalImage_Rectangle);

}

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

    cv::adaptiveThreshold(GaussGrayImage,BinaryImage,200,);

}
