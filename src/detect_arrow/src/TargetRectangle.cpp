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

    #ifdef DetectorRectangle_test
    cv::imshow("Adapted_PreProgress GreyImage",GreyImage);
    #endif

    cv::Mat GaussGrayImage;
    cv::GaussianBlur(GreyImage,
        GaussGrayImage,
        cv::Size(5,5),
        1.5
    );


    cv::Mat SharperImage;
    cv::convertScaleAbs(GaussGrayImage,SharperImage,DetectRectangleAlpha,DetectRectangleBeta);

    #ifdef DetectorRectangle_test
    cv::imshow("Adapted_PreProgress GaussGrayImage",GaussGrayImage);
    cv::imshow("Adapted_PreProgress SharperImage",SharperImage);
    #endif

    cv::adaptiveThreshold(SharperImage
        ,BinaryImage
        ,DetectRectangleMaxValue
        ,cv::ADAPTIVE_THRESH_MEAN_C,
        cv::THRESH_BINARY,
        DetectRectangleBlockSize,
        DetectRectangleC);

    #ifdef DetectorRectangle_test
    cv::imshow("Adapted_PreProgress BinaryImage",BinaryImage);
    #endif

    cv::Mat DilateImage;
    cv::Mat ErodedImage;
    cv::dilate(BinaryImage,
        DilatedImage,
        cv::getStructuringElement(cv::MorphShapes::MORPH_CROSS,
            TargetRectangleDilateCoreSize),
        cv::Point(-1, -1),
        TargetRectangleDilateItrations);

    cv::erode(DilatedImage,
        ErodedImage,
        cv::getStructuringElement(cv::MorphShapes::MORPH_CROSS,
            TargetRectangleErodeCoreSize),
            cv::Point(-1, -1),
            TargetRectangleErodeItrations);

    #ifdef DetectorRectangle_test
    cv::imshow("Adapted_PreProgress ErodedImage",ErodedImage);
    #endif

    #ifdef DetectorRectangle_test
    cv::imshow("Adapted_PreProgress DilatedImage",DilatedImage);
    cv::waitKey(11);
    #endif

    return ErodedImage;

}


std::vector<cv::Point2f> Arrow_detector::TargetRectangle(const cv::Mat & BinaryImage,cv::Mat & Image){
    static double eps=1e-9;
    Counters FirstCornerscounters,TargetCornerscounters;
    std::vector<cv::RotatedRect> CounterRotatedRects;
    std::vector<int> counter_size;
    cv::findContours(BinaryImage,FirstCornerscounters,cv::RETR_LIST,cv::CHAIN_APPROX_SIMPLE);

    for(auto &counter : FirstCornerscounters){

        int PixelNum;
        Counter approxresult;
        cv::RotatedRect rotatedrect;
        try{
            PixelNum=cv::contourArea(counter);
            rotatedrect=cv::minAreaRect(counter);
            cv::approxPolyDP(counter,approxresult,TargetRectangleapproxPolyDPepsilon,1);
        }
        catch(const std::exception& e){
            RCLCPP_WARN(this->get_logger(),"first loop TargetRectangle fail with %s",e.what());
            continue;
        }
        if(PixelNum<500) continue;

        //check 0
        if(std::abs(rotatedrect.size.height)<eps||std::abs(rotatedrect.size.width)<eps){
            continue;
        }
        double HWRate=std::max(rotatedrect.size.height,rotatedrect.size.width)/std::min(rotatedrect.size.height,rotatedrect.size.width);

        bool approxresultsizcheck= (TargetRectangleApproxSizeMin<=approxresult.size()&&approxresult.size()<=TargetRectangleApproxSizeMax);
        bool PixelNumcheck= (TargetRectanglePixelNumMin<=PixelNum&&PixelNum<=TargetRectanglePixelNumMax);
        bool HWRateCheck= (TargetRectangleHWRateMin<= HWRate && HWRate<= TargetRectangleHWRateMax);


        #ifdef DetectorRectangle_test_target
        RCLCPP_INFO(this->get_logger(),"approxresult.size= %ld",approxresult.size());
        RCLCPP_INFO( this->get_logger(),"PixelNum= %d",PixelNum);
        RCLCPP_INFO( this->get_logger(),"HWRate= %f",HWRate);

        RCLCPP_INFO(this->get_logger(),"TargetRectangle %s",(approxresultsizcheck&&PixelNumcheck&&HWRateCheck) ? "pass" : "fail");
        cv::drawContours(Image,std::vector<Counter>{counter},-1,cv::Scalar(255,34,123),3);
        cv::imshow("TargetRectangle__",Image);
        cv::waitKey(0);
        #endif


        if(!(approxresultsizcheck&&PixelNumcheck&&HWRateCheck)){
            continue;
        }

        std::vector<double> approxdis;
        for(std::size_t i=0;i<approxresult.size();i++){
            approxdis.push_back(DistancePoints(approxresult[i],approxresult[(i+1)%approxresult.size()]));
        }
        std::sort(approxdis.begin(),approxdis.end());
        double f2=(approxdis[0]+approxdis[1])/2;
        double e4=(approxdis[2]+approxdis[3]+approxdis[4]+approxdis[5])/4;
        double e4f2rate=e4/f2;

        bool e4f2ratecheck=(TargetRectanglee4f2rateMin<=e4f2rate&&e4f2rate<=TargetRectanglee4f2rateMax);

        if(!e4f2ratecheck){
            continue;
        }

        TargetCornerscounters.push_back(counter);
        counter_size.push_back(cv::contourArea(counter));

    
    }

    //clear some inbody counter
    Counters NoOverlapTargetCornerscounters;
    for(std::size_t i=0;i<TargetCornerscounters.size();i++){
        bool noverlap=true;
        for(std::size_t j=0;j<TargetCornerscounters.size();j++){
            if(i==j) continue;
            if(cv::pointPolygonTest(TargetCornerscounters[j],TargetCornerscounters[i][0],false)>=0){
                noverlap=false;
                break;
            }
        }
        if(noverlap){
            NoOverlapTargetCornerscounters.push_back(TargetCornerscounters[i]);
        }
    }

    // # ifdef DetectorRectangle_test_target
    for(auto & e : NoOverlapTargetCornerscounters){
        cv::drawContours(Image,std::vector<Counter>{e},-1,cv::Scalar(255,34,123),3);
        RCLCPP_INFO(this->get_logger(),"counter.size= %lf",cv::contourArea(e));
        cv::imshow("TargetRectangle",Image);
        // cv::waitKey(0);
    }
    // #endif
    //defult due to opencv the order of TargetCornerscounters is 逆时针顺序
    if(NoOverlapTargetCornerscounters.size()!=4){
        RCLCPP_WARN(this->get_logger(),"TargetCornerscounters.size()!=4 fail size= %ld",NoOverlapTargetCornerscounters.size());
        return std::vector<cv::Point2f>();
    }
    else{
        RCLCPP_INFO(this->get_logger(),"TargetCornerscounters.size()=4 pass size= %ld",NoOverlapTargetCornerscounters.size());
    }

    RCLCPP_INFO(this->get_logger(),"1");
    Counter AllPointSet;
    CombineCounters(NoOverlapTargetCornerscounters,AllPointSet);
    std::vector<Circle<float>> circles(4);
    Counter2fs TranglesOf3(4);
    Circle<float> allcircle;
    cv::minEnclosingCircle(AllPointSet,allcircle.center,allcircle.radius);

    // put the NoOverlapTargetCornerscounters in order
    std::sort(NoOverlapTargetCornerscounters.begin(),
        NoOverlapTargetCornerscounters.end(),
        [&allcircle](const Counter & a,const Counter & b){
            cv::Point2f a_,b_;
            float a_r,b_r;
            cv::minEnclosingCircle(a,a_,a_r);
            cv::minEnclosingCircle(b,b_,b_r);
            double anglea=GetAngleAccordingToHorizon(a_,allcircle.center);
            double angleb=GetAngleAccordingToHorizon(b_,allcircle.center);
            if(a_.y>allcircle.center.y) anglea+=180;
            if(b_.y>allcircle.center.y) angleb+=180;
            return (anglea<angleb);        
    });


    RCLCPP_INFO(this->get_logger(),"1");
    // 逆时针顺序
    Counter2f Corners(4);


    for(int i=0;i<4;i++){
        Counter combine3;
        CombineCounters(Counters{NoOverlapTargetCornerscounters[(i+3)%4],
            NoOverlapTargetCornerscounters[i],
            NoOverlapTargetCornerscounters[(i+1)%4]},combine3);
        cv::minEnclosingTriangle([&combine3](){
                Counter2f result;
                for(auto & e : combine3){
                    result.push_back(cv::Point2f(e.x,e.y));
                }
                return result;
            }(),TranglesOf3[i]);
        cv::minEnclosingCircle(NoOverlapTargetCornerscounters[i],circles[i].center,circles[i].radius);
        
        # ifdef DetectorRectangle_test_target
        cv::circle(Image,circles[i].center,circles[i].radius,cv::Scalar(255,34*i%225,123*i%225),1);
        DrawTrangle(Image,TranglesOf3[i],cv::Scalar(255,34*i%225,123*i%225),1);
        cv::imshow("TargetRectangle",Image);
        cv::waitKey(0);
        #endif

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
        cv::circle(Image,cv::Point(Corners[i].x,Corners[i].y),4,cv::Scalar(225,225,225),-1);
        cv::putText(Image,std::to_string(i),
            cv::Point(Corners[i].x,Corners[i].y),
            cv::FONT_HERSHEY_SIMPLEX,
            3.0,
            cv::Scalar(225,225,225));
    }
    cv::imshow("TargetRectangle",Image);
    #endif

    return Corners;
}

bool Arrow_detector::MainDetectArrow_Rectangle(const cv::Mat & OriginalImage){
    OriginalImage.copyTo(OriginalImage_Rectangle);
    cv::Mat BinaryImage=Adapted_PreProgress(OriginalImage_Rectangle);
    
    std::vector<cv::Point2f> TargetRectangleResult=TargetRectangle(BinaryImage,OriginalImage_Rectangle);

    if(TargetRectangleResult.size()==0) return 1;

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
        RCLCPP_INFO(this->get_logger(),"MainDetectArrow_Rectangle fail due to pnp");
        return 1;
    }

    #ifdef DetectorRectangle_test
    DrawPnPResult(OriginalImage_Rectangle,rvec,tvec,cv::Scalar(33,44,98),1,cv::Point(20,40));
    cv::imshow("MainDetectArrow_Rectangle",OriginalImage_Rectangle);
    cv::waitKey(11);
    #endif

    SendBoxPosition(tvec,rvec,OriginalImage_Rectangle);
    RCLCPP_INFO(this->get_logger(),"MainDetectArrow_Rectangle finish!");
    return 0;

}

void Arrow_detector::RectangleDetectorInit(){
    try{
        for(int i=0;i<4;i++){
            const std::vector<double> & RectanglePoints=config["RectanglePoints"][i].as<std::vector<double>>();
            RectangleOuterlayerCorners.push_back(cv::Point3d(RectanglePoints[0],RectanglePoints[1],RectanglePoints[2]));
            RectangleOuterlayerCornersEigen.push_back(Eigen::Vector4d(RectanglePoints[0],RectanglePoints[1],RectanglePoints[2],1));
        }
        for(int i=4;i<8;i++){
            const std::vector<double> & RectanglePoints=config["RectanglePoints"][i].as<std::vector<double>>();
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
        DetectRectangleMaxValue=config["rectangle_detect"]["DetectRectangleMaxValue"].as<double>();
        TargetRectanglee4f2rateMin=config["rectangle_detect"]["TargetRectanglee4f2rateMin"].as<double>();
        TargetRectanglee4f2rateMax=config["rectangle_detect"]["TargetRectanglee4f2rateMax"].as<double>();
        TargetRectangleErodeCoreSize=cv::Size(config["rectangle_detect"]["TargetRectangleErodeCoreSize"].as<std::vector<int>>()[0],
            config["rectangle_detect"]["TargetRectangleErodeCoreSize"].as<std::vector<int>>()[1]);
        TargetRectangleErodeItrations=config["rectangle_detect"]["TargetRectangleErodeItrations"].as<int>();
        TargetRectangleDilateItrations=config["rectangle_detect"]["TargetRectangleDilateItrations"].as<int>();
        TargetRectangleDilateCoreSize=cv::Size(config["rectangle_detect"]["TargetRectangleDilateCoreSize"].as<std::vector<int>>()[0],
            config["rectangle_detect"]["TargetRectangleDilateCoreSize"].as<std::vector<int>>()[1]);
        DetectRectangleAlpha=config["rectangle_detect"]["DetectRectangleAlpha"].as<double>();
        DetectRectangleBeta=config["rectangle_detect"]["DetectRectangleBeta"].as<double>();
    }
    catch(const std::exception& e){
        RCLCPP_ERROR(this->get_logger(),"Fail to load config file : %s",e.what());
        rclcpp::shutdown();
    }
    RCLCPP_INFO(this->get_logger(),"RectangleDetectorInit finish");
}