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
    static double eps=1e-9;
    Counters FirstCornerscounters,TargetCornerscounters;
    std::vector<cv::RotatedRect> CounterRotatedRects;
    std::vector<int> counter_size;
    cv::findContours(BinaryImage,FirstCornerscounters,cv::RETR_LIST,cv::CHAIN_APPROX_SIMPLE);

    for(auto &counter : FirstCornerscounters){
        Counter approxresult;
        cv::RotatedRect rotatedrect;
        cv::approxPolyDP(counter,approxresult,TargetRectangleapproxPolyDPepsilon,1);
        rotatedrect=cv::minAreaRect(counter);
        int PixelNum=cv::contourArea(counter);

        //check 0
        if(rotatedrect.size.height<eps||rotatedrect.size.width<eps){
            continue;
        }
        double HWRate=std::max(rotatedrect.size.height,rotatedrect.size.width)/std::min(rotatedrect.size.height,rotatedrect.size.width);

        bool approxresultsizcheck= (TargetRectangleApproxSizeMin<=approxresult.size()&&approxresult.size()<=TargetRectangleApproxSizeMax);
        bool PixelNumcheck= (TargetRectanglePixelNumMin<=PixelNum&&PixelNum<=TargetRectanglePixelNumMax);
        bool HWRateCheck= (TargetRectangleHWRateMin<= HWRate <= TargetRectangleHWRateMax);

        #ifdef DetectorRectangle_test_target

        cv::Mat Image_;
        Image.copyTo(Image_);
        RCLCPP_INFO(this->get_logger(),"TargetRectangle: approxresult %ld",approxresult.size());
        RCLCPP_INFO(this->get_logger(),"TargetRectangle: PixelNum %ld", PixelNum);
        RCLCPP_INFO(this->get_logger(),"TargetRectangle: HWRate %lf",HWRate);
        RCLCPP_INFO(this->get_logger(),"TargetRectangle %s",(approxresultsizcheck&&PixelNumcheck&&HWRateCheck) ? "pass" : "nopass");
        cv::drawContours(Image_,Counters{counter},-1,cv::Scalar(223,89,54),1);
        cv::imshow("DetectorRectangle_test_target",Image_);
        cv::waitKey(0);

        #endif

        if(!(approxresultsizcheck&&PixelNumcheck&&HWRateCheck)){
            continue;
        }

        //need more

        TargetCornerscounters.push_back(counter);
        counter_size.push_back(cv::contourArea(counter));
    }
    //defult due to opencv the order of TargetCornerscounters is 逆时针顺序
    if(TargetCornerscounters.size()!=4){
        RCLCPP_WARN(this->get_logger(),"TargetCornerscounters.size()!=4 fail");
        return std::vector<cv::Point2f>();
    }
    
    sort(counter_size.begin(),counter_size.end());
    if(counter_size[0]<counter_size[3]){
        RCLCPP_WARN(this->get_logger(),"TargetCornerscounters maxsize/minsize>=10 too large");
        return std::vector<cv::Point2f>();
    }
    
    Counter AllPointSet;
    CombineCounters(TargetCornerscounters,AllPointSet);
    std::vector<Circle<float>> circlesOf3(4),circles(4);
    Counter2fs TranglesOf3(4);

    // 逆时针顺序
    Counter2f Corners(4);


    for(int i=0;i<4;i++){
        Counter combine3;
        CombineCounters(Counters{TargetCornerscounters[(i+3)%4],
            TargetCornerscounters[i],
            TargetCornerscounters[(i+1)%4]},combine3);
        cv::minEnclosingCircle(combine3,circlesOf3[i].center,circlesOf3[i].radius);
        cv::minEnclosingTriangle(combine3,TranglesOf3[i]);
        cv::minEnclosingCircle(TargetCornerscounters[i],circles[i].center,circles[i].radius);

        double dis=1e9;
        for(auto & e : TranglesOf3[i]){
            if(DistancePoints(e,circles[i].center)<dis){
                dis=DistancePoints(e,circles[i].center);
                Corners[i]=e;
            }
        }
    }


    #ifdef DetectorRectangle_test
    for(int i=0;i<4;i++){
        cv::circle(Image,cv::Point(Corners[i].x,Corners[i].y),1,cv::Scalar(43,23,100),-1);
        cv::putText(Image,std::to_string(i),
            cv::Point(Corners[i].x,Corners[i].y),
            cv::FONT_HERSHEY_SIMPLEX,
            1.0,
            cv::Scalar(43,23,100));
    }
    cv::imshow("TargetRectangle",Image);
    #endif

    return Corners;
}

bool Arrow_detector::MainDetectArrow_Rectangle(const cv::Mat & OriginalImage){
    OriginalImage.copyTo(OriginalImage_Rectangle);
    cv::Mat BinaryImage=Adapted_PreProgress(OriginalImage_Rectangle);
    
    std::vector<cv::Point2f> TargetRectangleResult=TargetRectangle(BinaryImage,OriginalImage_Rectangle);

    Counter2d TargetRectangleResult2d;
    for(auto & i : TargetRectangleResult) TargetRectangleResult2d.push_back(cv::Point2d(i.x,i.y));

    cv::Mat tvec,rvec;

    bool PnPsolverCheck=PnPsolver(TargetRectangleResult2d,
        RectangleOuterlayerCorners,
        cameraMatrix,
        distCoeffs,
        rvec,
        tvec,
        0,
        cv::SOLVEPNP_IPPE);
    
    if(!PnPsolverCheck){
        return 1;
    }

    DrawPnPResult(OriginalImage_Rectangle,rvec,tvec,cv::Scalar(33,44,98),1,cv::Point(20,40));
    
    SendBoxPosition(tvec,rvec,OriginalImage_Rectangle);
    
    return 0;

}

void Arrow_detector::RectangleDetectorInit(){
    // YAML::Node configRectangle=
    try{
        for(int i=0;i<4;i++){
            const std::vector<double> & RectanglePoints=config["arrow"]["RectanglePoints"][i].as<std::vector<double>>();
            RectangleOuterlayerCorners.push_back(cv::Point3d(RectanglePoints[0],RectanglePoints[1],RectanglePoints[2]));
            RectangleOuterlayerCornersEigen.push_back(Eigen::Vector4d(RectanglePoints[0],RectanglePoints[1],RectanglePoints[2],1));
        }
        for(int i=4;i<8;i++){
            const std::vector<double> & RectanglePoints=config["redeem_box"]["RectanglePoints"][i].as<std::vector<double>>();
            RectangleInnerlayerCorners.push_back(cv::Point3d(RectanglePoints[0],RectanglePoints[1],RectanglePoints[2]));
            RectangleInnerlayerCornersEigen.push_back(Eigen::Vector4d(RectanglePoints[0],RectanglePoints[1],RectanglePoints[2],1));
        }

        DetectRectangleBlockSize=config["rectangle_detect"]["DetectRectangleBlockSize"].as<int>();
        DetectRectangleC=config["rectangle_detect"]["DetectRectangleC"].as<double>();
        TargetRectangleapproxPolyDPepsilon=config["rectangle_detect"]["TargetRectangleapproxPolyDPepsilon"].as<double>();
        TargetRectangleApproxSizeMin=config["rectangle_detect"]["TargetRectangleApproxSizeMin"].as<int>();
        TargetRectangleApproxSizeMax=config["rectangle_detect"]["TargetRectangleApproxSizeMax"].as<int>();
        TargetRectanglePixelNumMin=config["rectangle_detect"]["TargetRectanglePixelNumMin"].as<int>();
        TargetRectanglePixelNumMax=config["rectangle_detect"]["TargetRectanglePixelNumMax"].as<int>();
        TargetRectangleHWRateMin=config["rectangle_detect"]["TargetRectangleHWRateMin"].as<double>();
        TargetRectangleHWRateMax=config["rectangle_detect"]["TargetRectangleHWRateMax"].as<double>();
    }
    catch(const std::exception& e){
        RCLCPP_ERROR(this->get_logger(),"Fail to load config file : %s",e.what());
        rclcpp::shutdown();
    }
}