#include "DetectArrow.hpp"

/*
[INFO] [1736395125.791977894] [calibrate_camera]: cameraMatrix : [2585.088699294729, 0, 525.0062538082545;
 0, 2643.218336464573, 633.0861170979923;
 0, 0, 1]
distCoeffs : [-0.0556364495907907, -0.2212500754240332, 0.007170186529455457, 0.00956399082050834, 1.912809759877645]

[INFO] [1736395298.474409751] [calibrate_camera]: cameraMatrix : [2381.944332254315, 0, 709.3373659842438;
 0, 2387.166170211481, 594.0678751202787;
 0, 0, 1]
distCoeffs : [-0.07414935707790817, 0.3823790114920249, 0.003279745743046585, 0.001628738510793524, -0.9057026792700137]

[INFO] [1736395505.051047581] [calibrate_camera]: cameraMatrix : [2382.895292065366, 0, 710.5726099864352;
 0, 2390.181509542392, 596.0300870268205;
 0, 0, 1]
distCoeffs : [-0.07221877942119345, 0.2013289769688859, 0.003024436349887122, 0.002969241447513818, 0.4220593044244383]

[INFO] [1736395698.023824032] [calibrate_camera]: cameraMatrix : [2387.558408527406, 0, 715.0159139648023;
 0, 2393.04251032088, 594.0275525380879;
 0, 0, 1]
distCoeffs : [-0.06833047900817182, 0.2942394128667126, 0.002870541026948472, 0.002410173251176433, 0.1036437823223326]

[INFO] [1736395761.683406313] [calibrate_camera]: cameraMatrix : [2375.260793164691, 0, 723.0037072901363;
 0, 2380.732591475137, 605.9553853268534;
 0, 0, 1]
distCoeffs : [-0.06950109379156319, 0.2968388146211183, 0.004616267963532699, 0.003975553594638102, -0.5707120681301413]

[INFO] [1736395843.989932800] [calibrate_camera]: cameraMatrix : [2366.778471623255, 0, 748.0916761127738;
 0, 2369.630431231162, 603.8068306938617;
 0, 0, 1]
distCoeffs : [-0.08923122531565172, 0.5371551780954817, 0.003580395486770035, 0.004370536786784852, -1.844051438880483]

[INFO] [1736397778.916076636] [calibrate_camera]: cameraMatrix : [2360.008613990164, 0, 761.9832997464299;
 0, 2362.751874116515, 596.5809202479236;
 0, 0, 1]
distCoeffs : [-0.07265408826672405, 0.1511829681751362, 0.003579730882815966, 0.00652837881894829, 0.7284753107946278]

[INFO] [1736397822.324780480] [calibrate_camera]: cameraMatrix : [2369.24973891433, 0, 745.0359718145587;
 0, 2373.023758080285, 592.3692178752259;
 0, 0, 1]
distCoeffs : [-0.0791587761424231, 0.3446287126868074, 0.003336949690923318, 0.005161324417881087, -0.2043942064847434]

[INFO] [1736397847.257374538] [calibrate_camera]: cameraMatrix : [2365.58446740118, 0, 735.3589551516543;
 0, 2369.87147521186, 597.3784330607212;
 0, 0, 1]
distCoeffs : [-0.08304354728867848, 0.4740520671349235, 0.003425146248158939, 0.003879155461288919, -1.4157542645932]

[INFO] [1736397877.122511162] [calibrate_camera]: cameraMatrix : [2375.787121776882, 0, 740.0689192411256;
 0, 2379.743671056914, 590.1717549829305;
 0, 0, 1]
distCoeffs : [-0.07093428455159315, 0.1865900206019591, 0.003095286426499801, 0.004747807496693957, 0.8787773017757813]

*/

std::vector<Eigen::Matrix<double,4,1>> ObjRedemptionBoxCornerPointEigen={
    Eigen::Matrix<double,4,1>(-117.02617228637361528833974192835,52.679455198397790567862904976811,24,1),
    Eigen::Matrix<double,4,1>(52.679455198397790567862904976811,-117.02617228637361528833974192835,24,1),
    Eigen::Matrix<double,4,1>(52.679455198397790567862904976811,-117.02617228637361528833974192835,264,1),
    Eigen::Matrix<double,4,1>(-117.02617228637361528833974192835,52.679455198397790567862904976811,264,1)
};

std::vector<Eigen::Matrix<double,4,1>> Object2cornersEigen={
    Eigen::Matrix<double,4,1>(-133.99673503485076,69.65001794687493,0,1),
    Eigen::Matrix<double,4,1>(69.65001794687493,-133.99673503485076,0,1),
};

typedef std::vector<std::vector<cv::Point>> Counters;
typedef std::vector<cv::Point> Counter;

std::mutex mtxvideoget;

const long double eps=1e-9;

struct Slope{
    int p1,p2;
    double slope;
};

double GetAngleAccordingToHorizon(cv::Point p1,cv::Point p2){
    cv::Point horison(1,0),tar=p1-p2;
    if(tar.y>=eps) tar=-tar;
    double dot_=tar.dot(horison);
    double cos_=dot_/(cv::norm(tar)*cv::norm(horison));
    double angle=std::acos(cos_)/CV_PI*180;
    return angle;
}

double GetAngle(cv::Point2f p1,cv::Point2f p2,cv::Point2f p3){
    cv::Point2f vec1=p2-p1,vec2=p3-p1;
    double dot_=vec1.dot(vec2);
    double cos_=dot_/(cv::norm(vec2)*cv::norm(vec1));
    double angle=std::acos(cos_)/CV_PI*180;
    return angle;
}

double DistancePoints(const cv::Point & p1,const cv::Point & p2){
    return std::sqrt((p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y));
}

double DistancePoints(const cv::Point2f & p1,const cv::Point & p2){
    return std::sqrt((p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y));
}

double DistancePoints(const cv::Point & p1,const cv::Point2f & p2){
    return std::sqrt((p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y));
}

bool Arrow_detector::PnPsolver(){
    cv::Mat cameraMatrixCV=cv::Mat(3,3,CV_64F,const_cast<double *>(cameraMatrix.data())).clone();
    cv::Mat distCoeffsCV=cv::Mat(1,5,CV_64F,const_cast<double *>(distCoeffs.data())).clone();
    Eigen::Matrix<double,3,3> cameraMatrixEigen;
    Eigen::Matrix<double,4,4> rtvecEigen;
    Eigen::Matrix<double,3,4> signMat;

    bool PnPsuccess=cv::solvePnP(objpoints,ArrowPeaks,cameraMatrixCV,distCoeffsCV,rvec,tvec,false,cv::SOLVEPNP_IPPE);

    if(!PnPsuccess){
        RCLCPP_WARN(this->get_logger(),"PnP fail");
        return 0;
    }

    std::stringstream ss;
    ss<<"rvec:\n"<<rvec<<std::endl;
    ss<<"tvec:\n"<<tvec<<std::endl;
    RCLCPP_INFO(this->get_logger(),"%s",ss.str().c_str());
    RCLCPP_INFO(this->get_logger(),"finish pnp");

    for(int i=0;i<3;i++){
        for(int e=0;e<3;e++){
            cameraMatrixEigen(i,e)=cameraMatrix[i*3+e];
        }
    }

    signMat<<1,0,0,0,
        0,1,0,0,
        0,0,1,0;

    cv::Mat rmat(rvec.size(),rvec.type());
    cv::Rodrigues(rvec,rmat);


    for(int i=0;i<3;i++){
        for(int e=0;e<3;e++){
            rtvecEigen(i,e)=rmat.at<double>(i,e);
        }
        rtvecEigen(i,3)=tvec.at<double>(i);
    }
    
    rtvecEigen(3, 0) = 0.0;
    rtvecEigen(3, 1) = 0.0;
    rtvecEigen(3, 2) = 0.0;
    rtvecEigen(3, 3) = 1.0;


    std::stringstream sss;
    sss<<rtvecEigen;
    RCLCPP_INFO(this->get_logger(),"rtvecEigen: %s",sss.str().c_str());

    ImageRedemptionBoxCornerPoints.clear();
    for(const auto & i : ObjRedemptionBoxCornerPointEigen){
        auto coordination=cameraMatrixEigen*signMat*rtvecEigen*i;
        ImageRedemptionBoxCornerPoints.push_back(cv::Point2i(coordination(0)/coordination(2),coordination(1)/coordination(2)));
        
        std::stringstream ss;
        ss<<cameraMatrixEigen<<"\n"<<signMat<<"\n"<<rtvecEigen<<"\n"<<i<<"\n"<<coordination;
        RCLCPP_INFO(this->get_logger(),"Node : %s",ss.str().c_str());


    }


// #ifdef DeBug

    Counter corners;
    for(const auto & i : Object2cornersEigen){
        auto coordination=cameraMatrixEigen*signMat*rtvecEigen*i;
        corners.push_back(cv::Point2i(coordination(0)/coordination(2),coordination(1)/coordination(2)));
        
        std::stringstream ss;
        ss<<cameraMatrixEigen<<"\n"<<signMat<<"\n"<<rtvecEigen<<"\n"<<i<<"\n"<<coordination;
        RCLCPP_INFO(this->get_logger(),"Node : %s",ss.str().c_str());

    }
    cv::drawContours(OriginalImage,Counters{corners},-1,cv::Scalar(225,0,0),5);

    cv::drawContours(OriginalImage,Counters{ImageRedemptionBoxCornerPoints},-1,cv::Scalar(225,0,0),3);

    // cv::putText(OriginalImage,ss.str().c_str(),cv::Point(0,0),cv::FONT_HERSHEY_SIMPLEX,1.0,cv::Scalar(225,0,0));

// #endif

    cv::imshow("PnP",OriginalImage);
    cv::waitKey(0);

    static int CntVideo=0;
    static bool close=0;
    if(CntVideo<=60*30){
        videowriter<<OriginalImage;
        CntVideo++;
        RCLCPP_INFO(this->get_logger(),"wirte video %d",CntVideo);
    }
    else if(!close){
        videowriter.release();
        close=1;
    }

    RCLCPP_INFO(this->get_logger(),"PnPsolver finish");

    return 1;
}

void Arrow_detector::CreatGetImageTimer(){
    using namespace std::placeholders;
    using namespace std::chrono;
    node_shred_ptr=this->shared_from_this();
    cv::namedWindow("ArrowPeaks");
    cv::namedWindow("Dilated");
    cv::namedWindow("PnP");
    // RCLCPP_INFO(this->get_logger(),"Start Creat Get Image Timer.");
    // timer_=this->create_wall_timer(33ms,std::bind(&Arrow_detector::GetImage,this));
    // RCLCPP_INFO(this->get_logger(),"Creat Get Image Timer.");
    // this->GetImage();
}

void Arrow_detector::GetImage(const sensor_msgs::msg::Image::SharedPtr msg){
    using namespace std::chrono;
    // mtxvideoget.lock();
    // while(!client_->wait_for_service(1s)){
    //     if(!rclcpp::ok()){
    //         RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
    //         rclcpp::shutdown();
    //         return;
    //     }
    //     RCLCPP_INFO(this->get_logger(),"service not available, waiting again....");
    // }

    // auto request=std::make_shared<Imagerequest::Request>();
    // auto result=client_->async_send_request(request);
    // RCLCPP_INFO(this->get_logger(),"Send request.");
    // sensor_msgs::msg::Image msg;

    // if(rclcpp::spin_until_future_complete(node_shred_ptr,result)==rclcpp::FutureReturnCode::SUCCESS){
    // // if(result.wait_for(1s)!=std::future_status::timeout){
    //     RCLCPP_INFO(this->get_logger(),"!!!!!");
    //     auto result_=result.get();
    //     if(result_->end){
    //         RCLCPP_INFO(this->get_logger(),"Video comes to end.");
    //         return;
    //     }
    //     msg=result_->image;
    //     RCLCPP_INFO(this->get_logger(),"Get frame with size [%d,%d]",msg.height,msg.width);
    // }
    // else {
    //     RCLCPP_ERROR(this->get_logger(), "Wait too long");
    //     return;
    // }

    if(!rclcpp::ok()){
        rclcpp::shutdown();
    }

    cv_bridge::CvImagePtr cv_ptr;
    try{
        cv_ptr=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception& e){
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat originalframe=cv_ptr->image;
    originalframe.copyTo(OriginalImage);
    RCLCPP_INFO(this->get_logger(), "Get frame");
    // cv::imshow("Video",originalframe);
    // cv::waitKey(33);
    MainDetectArrow(originalframe);

    // mtxvideoget.unlock();
}

bool Arrow_detector::TargetArrow(const cv::Mat & BinaryImage){

    // static int maxn=0,minn=1e9,approsiz=0;
    // static bool pre=0;

    std::vector<cv::Vec2f> lines;
    Counters counters_;
    Counter isarrow;
    Counter arrowapproxcurve;
    
    cv::findContours(BinaryImage,counters_,cv::RETR_LIST,cv::CHAIN_APPROX_SIMPLE);

    // cv::drawContours(OriginalImage,counters_,-1,cv::Scalar(225,0,0),1);

    // cv::imshow("edge1",OriginalImage);
    // cv::imshow("edge",edge);
    // cv::waitKey(33);

    for(auto &counter_ :counters_){

        cv::RotatedRect rotatedrect_=cv::minAreaRect(counter_);
        Counter approxcurve;

        double LengthWidthRatio= (std::min(rotatedrect_.size.width,rotatedrect_.size.height)<=eps ? 
            -1 : std::max(rotatedrect_.size.width,rotatedrect_.size.height)/std::min(rotatedrect_.size.width,rotatedrect_.size.height));
        int pixel_num=cv::contourArea(counter_);
        cv::approxPolyDP(counter_,approxcurve,20,1);

        RCLCPP_INFO(this->get_logger(),"LengthWidthRatio : %lf",LengthWidthRatio);
        RCLCPP_INFO(this->get_logger(),"pixel_num : %d",pixel_num);
        RCLCPP_INFO(this->get_logger(),"approxcurve size : %ld",approxcurve.size());

        bool pixel_in=(pixel_num>=15000&&pixel_num<=80000);
        bool lwratio=1.1<=LengthWidthRatio&&LengthWidthRatio<=(4/sqrt(2));
        bool approxsize=(6<=approxcurve.size()&&approxcurve.size()<=10);

        if(pixel_in&&lwratio&&approxsize){
            isarrow=counter_;
            arrowapproxcurve=approxcurve;
            break;
        }
    }

    if(isarrow.empty()){
        RCLCPP_WARN(this->get_logger(),"fail to find arrow!");
        return 0;
    }
    else RCLCPP_INFO(this->get_logger(),"find arrow!");

    cv::Point2f center;
    std::vector<cv::Point2f> TrianglePeaks;
    float radius;
    cv::Mat MaskedImage;
    cv::Mat Mask(OriginalImage.size(),CV_8UC1,cv::Scalar(0));
    cv::minEnclosingCircle(isarrow,center,radius);
    cv::minEnclosingTriangle(isarrow,TrianglePeaks);

    cv::circle(Mask,center,radius,cv::Scalar(255),-1);

    cv::copyTo(BinaryImage,MaskedImage,Mask);

    #ifdef DeBugHough

    cv::imshow("MaskedImage",MaskedImage);
    cv::imshow("BinaryImage",BinaryImage);
    cv::imshow("Mask",Mask);
    cv::waitKey(33);

    #endif

    #ifdef DeBug

    cv::circle(OriginalImage,center,radius,cv::Scalar(225,0,225));
    cv::circle(OriginalImage,center,2,cv::Scalar(225,0,225),-1);
    cv::line(OriginalImage,TrianglePeaks[0],TrianglePeaks[1],cv::Scalar(0,225,225));
    cv::line(OriginalImage,TrianglePeaks[0],TrianglePeaks[2],cv::Scalar(0,225,225));
    cv::line(OriginalImage,TrianglePeaks[1],TrianglePeaks[2],cv::Scalar(0,225,225));
    cv::drawContours(OriginalImage,Counters{isarrow},-1,cv::Scalar(225,225,0));
    cv::drawContours(OriginalImage,Counters{arrowapproxcurve},-1,cv::Scalar(150,225,150));

    #endif

    double maxAngle=0;
    cv::Point2f TrianglePeak;
    for(int i=0;i<3;i++){
        if(maxAngle<GetAngle(TrianglePeaks[i],TrianglePeaks[(i+1)%3],TrianglePeaks[(i+2)%3])){
            TrianglePeak=TrianglePeaks[i];
            maxAngle=GetAngle(TrianglePeaks[i],TrianglePeaks[(i+1)%3],TrianglePeaks[(i+2)%3]);
        }
    }

    // cv::circle(OriginalImage,TrianglePeak,2,cv::Scalar(225,0,225),-1);
    // cv::imshow("11111",OriginalImage);
    // cv::waitKey(33);

    std::vector<Slope> slopes;
    double HorizonThreshold=10,RThreshold=20,LThreshold=0;
    int TryCnt=0;
    std::vector<bool> UsedLineMakePair(20,false);
    std::vector<std::pair<Slope,Slope>> HorizonLinePair;

    for(int i=arrowapproxcurve.size()-1,siz=arrowapproxcurve.size();i>=0;i--){
        slopes.push_back(Slope{i,(i+1)%siz,GetAngleAccordingToHorizon(arrowapproxcurve[i],arrowapproxcurve[(i+1)%siz])});

        #ifdef DeBug
        // cv::line(OriginalImage,arrowapproxcurve[i],arrowapproxcurve[e],cv::Scalar(225,225,225));
        std::stringstream ss;
        ss<<std::fixed<<std::setprecision(2)<<GetAngleAccordingToHorizon(arrowapproxcurve[i],arrowapproxcurve[(i+1)%siz]);
        cv::putText(OriginalImage,ss.str(),(arrowapproxcurve[i]+arrowapproxcurve[(i+1)%siz])/2,cv::FONT_HERSHEY_SIMPLEX,1.0,cv::Scalar(115,216,22));
        #endif

        RCLCPP_INFO(this->get_logger(),"angle: %lf",GetAngleAccordingToHorizon(arrowapproxcurve[i],arrowapproxcurve[(i+1)%siz]));
    }

    std::sort(slopes.begin(),slopes.end(),[](const Slope & a,const Slope & b){
        return a.slope<b.slope;
    });

    // cv::imshow("lll",OriginalImage);
    // cv::waitKey(33);

    while(HorizonLinePair.size()!=3&&TryCnt<=20){
        HorizonLinePair.clear();
        for(int i=slopes.size()-1;i>=0;i--) UsedLineMakePair[i]=0;

        HorizonThreshold=(RThreshold+LThreshold)/2;
        for(int i=slopes.size()-1;i;i--){
            if(UsedLineMakePair[i]||std::abs(slopes[i].slope-slopes[i-1].slope)>HorizonThreshold) continue;
            HorizonLinePair.push_back(std::make_pair(slopes[i],slopes[i-1]));
            UsedLineMakePair[i]=UsedLineMakePair[i-1]=1;
        }
        if(HorizonLinePair.size()==3) break;
        if(HorizonLinePair.size()<3) LThreshold=HorizonThreshold;
        else if(HorizonLinePair.size()>3) RThreshold=HorizonThreshold;
        TryCnt++;
    }

    if(HorizonLinePair.size()!=3){
        RCLCPP_WARN(this->get_logger(),"fail to find horizon line pairs");
        return 0;
    }
    else RCLCPP_INFO(this->get_logger(),"find horizon line pairs!");

    std::sort(HorizonLinePair.begin(),HorizonLinePair.end(),[&arrowapproxcurve](const std::pair<Slope,Slope>& a,const std::pair<Slope,Slope>& b){
        return  DistancePoints(arrowapproxcurve[a.first.p1],arrowapproxcurve[a.first.p2])+DistancePoints(arrowapproxcurve[a.second.p1],arrowapproxcurve[a.second.p2]) >
            DistancePoints(arrowapproxcurve[b.first.p1],arrowapproxcurve[b.first.p2])+DistancePoints(arrowapproxcurve[b.second.p1],arrowapproxcurve[b.second.p2]);
    });

    std::vector<int> CountPoint(isarrow.size(),0);
    std::vector<int> RightAnglePeaks;
    
    for(int i=0;i<=1;i++){
        CountPoint[HorizonLinePair[i].first.p1]++;
        CountPoint[HorizonLinePair[i].first.p2]++;
        CountPoint[HorizonLinePair[i].second.p1]++;
        CountPoint[HorizonLinePair[i].second.p2]++;
        if(CountPoint[HorizonLinePair[i].first.p1]==2) RightAnglePeaks.push_back(HorizonLinePair[i].first.p1);
        if(CountPoint[HorizonLinePair[i].first.p2]==2) RightAnglePeaks.push_back(HorizonLinePair[i].first.p2);
        if(CountPoint[HorizonLinePair[i].second.p1]==2) RightAnglePeaks.push_back(HorizonLinePair[i].second.p1);
        if(CountPoint[HorizonLinePair[i].second.p2]==2) RightAnglePeaks.push_back(HorizonLinePair[i].second.p2);
    }

    if(RightAnglePeaks.size()!=2){
        RCLCPP_WARN(this->get_logger(),"RightAnglePeaks.size != 2");

        #ifdef DeBug
        // cv::line(OriginalImage,arrowapproxcurve[HorizonLinePair[0].first.p1],arrowapproxcurve[HorizonLinePair[0].first.p2],cv::Scalar(100,200,150),5);
        // cv::line(OriginalImage,arrowapproxcurve[HorizonLinePair[0].second.p1],arrowapproxcurve[HorizonLinePair[0].second.p2],cv::Scalar(100,200,150),5);
        // cv::line(OriginalImage,arrowapproxcurve[HorizonLinePair[1].first.p1],arrowapproxcurve[HorizonLinePair[1].first.p2],cv::Scalar(50,220,225),5);
        // cv::line(OriginalImage,arrowapproxcurve[HorizonLinePair[1].second.p1],arrowapproxcurve[HorizonLinePair[1].second.p2],cv::Scalar(50,220,225),5);
        // cv::imshow("Fail",OriginalImage);
        // cv::waitKey(0);
        #endif

        return 0;
    }
    else RCLCPP_INFO(this->get_logger(),"finish dichotomy and find two right angle peaks");

    if(DistancePoints(center,arrowapproxcurve[RightAnglePeaks[0]])<DistancePoints(center,arrowapproxcurve[RightAnglePeaks[1]])){
        std::swap(RightAnglePeaks[0],RightAnglePeaks[1]);
    }
    //确定第一个为外侧点，第二个为内侧点

    RCLCPP_INFO(this->get_logger(),"find right angle !");

    #ifdef DeBug

    // cv::circle(OriginalImage,center,radius,cv::Scalar(0,225,225),1);
    cv::circle(OriginalImage,arrowapproxcurve[RightAnglePeaks[0]],1,cv::Scalar(112,233,200),-1);
    cv::circle(OriginalImage,arrowapproxcurve[RightAnglePeaks[1]],1,cv::Scalar(112,233,200),-1);

    // cv::imshow("lll",OriginalImage);
    // cv::waitKey(33);
    #endif

    // RCLCPP_INFO(this->get_logger(),"OK!");

    /*
    储存规则：
    最外侧直角顶点，最内侧直角顶点，外接圆顺时针方向第一个尾处的两顶点的中点，外接圆顺时针方向第二个尾处的两顶点的中点
    */
    std::vector<cv::Point> ArrowPeaks;

    ArrowPeaks.push_back(arrowapproxcurve[RightAnglePeaks[0]]);
    ArrowPeaks.push_back(arrowapproxcurve[RightAnglePeaks[1]]);
    ArrowPeaks.push_back(cv::Point(0,0));
    ArrowPeaks.push_back(cv::Point(0,0));


    cv::Point Centerline=ArrowPeaks[0]-cv::Point(center.x,center.y);
    for(int i=0;i<=1;i++){
        std::pair<int,int> PointNumPair1=std::make_pair(HorizonLinePair[i].first.p1,HorizonLinePair[i].first.p2);
        std::pair<int,int> PointNumPair2=std::make_pair(HorizonLinePair[i].second.p1,HorizonLinePair[i].second.p2);
        if(PointNumPair1.first==RightAnglePeaks[0]||PointNumPair1.first==RightAnglePeaks[1]) std::swap(PointNumPair1.first,PointNumPair1.second);
        if(PointNumPair2.first==RightAnglePeaks[0]||PointNumPair2.first==RightAnglePeaks[1]) std::swap(PointNumPair2.first,PointNumPair2.second);
        cv::Point TargetLine=arrowapproxcurve[PointNumPair1.first]-cv::Point(center.x,center.y);
        if(TargetLine.cross(Centerline)<=0) ArrowPeaks[2]=(arrowapproxcurve[PointNumPair1.first]+arrowapproxcurve[PointNumPair2.first])/2;
        else ArrowPeaks[3]=(arrowapproxcurve[PointNumPair1.first]+arrowapproxcurve[PointNumPair2.first])/2;
    }

    if(ArrowPeaks[3]==cv::Point(0,0)||ArrowPeaks[2]==cv::Point(0,0)){
        RCLCPP_ERROR(this->get_logger(),"Fail to find midpoint of other two sides");
        return 0;
    }
    else RCLCPP_INFO(this->get_logger(),"find midpoint of other two sides successfully");

    #ifdef DeBug

    int PeaksCnt=0;
    for(auto i : ArrowPeaks){
        cv::circle(OriginalImage,i,1,cv::Scalar(153,156,30),-1);
        std::stringstream ss;ss<<PeaksCnt<<":"<<(i-cv::Point(center)).cross(Centerline);PeaksCnt++;
        cv::putText(OriginalImage,ss.str(),i,cv::FONT_HERSHEY_SIMPLEX,1.0,cv::Scalar(225,225,225));
    }

    cv::imshow("ArrowPeaks",OriginalImage);
    cv::waitKey(22);


    for(auto i : ArrowPeaks){
        cv::circle(OriginalImage,i,1,cv::Scalar(153,156,30),-1);
        // std::stringstream ss;ss<<PeaksCnt<<":"<<(i-cv::Point(center)).cross(Centerline);PeaksCnt++;
        // cv::putText(OriginalImage,ss.str(),i,cv::FONT_HERSHEY_SIMPLEX,1.0,cv::Scalar(225,225,225));
    }

    #endif

    Lines ArrowLines;
    cv::Mat CannyImage;
    cv::Canny(MaskedImage,CannyImage,100,200);

    cv::HoughLines(CannyImage,ArrowLines,1,CV_PI/360,200);

    std::vector<std::pair<int,double>> DistanceBTOutCornerLines;
    std::vector<std::pair<int,double>> DistanceBTInCornerLines;

    for(int i=ArrowLines.size()-1;i>=0;i--){
        DistanceBTOutCornerLines.push_back(std::make_pair(i,DistanceBetweenPointAndLine(ArrowPeaks[0],ArrowLines[i])));
        DistanceBTInCornerLines.push_back(std::make_pair(i,DistanceBetweenPointAndLine(ArrowPeaks[1],ArrowLines[i])));
    }

    std::sort(DistanceBTOutCornerLines.begin(),DistanceBTOutCornerLines.end(),[](const std::pair<int,double> & a,const std::pair<int,double> & b){
        return a.second<b.second;
    });

    std::sort(DistanceBTInCornerLines.begin(),DistanceBTInCornerLines.end(),[](const std::pair<int,double> & a,const std::pair<int,double> & b){
        return a.second<b.second;
    });

    const int SizeofLines=ArrowLines.size();
    const double ParallelThreshold=CV_PI/6;

    if(SizeofLines<2){
        RCLCPP_WARN(this->get_logger(),"ArrowLines need at least 2 lines");
        return 0;
    }
    else RCLCPP_INFO(this->get_logger(),"ArrowLines has %d lines",SizeofLines);

    for(int i=1;i<SizeofLines;i++){
        RCLCPP_INFO(this->get_logger(),"angle 0 : [%lf], angle %d : [%lf]",ArrowLines[DistanceBTOutCornerLines[0].first].val[1],i,ArrowLines[DistanceBTOutCornerLines[i].first].val[1]);
        if(std::abs(ArrowLines[DistanceBTOutCornerLines[0].first].val[1]-ArrowLines[DistanceBTOutCornerLines[i].first].val[1])<ParallelThreshold) continue;
        RCLCPP_INFO(this->get_logger(),"change two lines");
        if(1!=i) std::swap(DistanceBTOutCornerLines[1],DistanceBTOutCornerLines[i]);
        break;
    }

    for(int i=1;i<SizeofLines;i++){
        RCLCPP_INFO(this->get_logger(),"angle 0 : [%lf], angle %d : [%lf]",ArrowLines[DistanceBTInCornerLines[0].first].val[1],i,ArrowLines[DistanceBTInCornerLines[i].first].val[1]);
        if(std::abs(ArrowLines[DistanceBTInCornerLines[0].first].val[1]-ArrowLines[DistanceBTInCornerLines[i].first].val[1])<ParallelThreshold) continue;
        RCLCPP_INFO(this->get_logger(),"change two lines");
        if(1!=i) std::swap(DistanceBTInCornerLines[1],DistanceBTInCornerLines[i]);
        break;
    }

    if(std::abs(ArrowLines[DistanceBTInCornerLines[0].first].val[1]-ArrowLines[DistanceBTInCornerLines[1].first].val[1])<ParallelThreshold||
        std::abs(ArrowLines[DistanceBTOutCornerLines[0].first].val[1]-ArrowLines[DistanceBTOutCornerLines[1].first].val[1])<ParallelThreshold){
        RCLCPP_WARN(this->get_logger(),"fail to find two lines");
        return 0;
    }
    else RCLCPP_INFO(this->get_logger(),"find two lines");

    //End part

    #ifdef DeBugHough

    DrawLines(OriginalImage,ArrowLines,cv::Scalar(225,225,225));

    for(auto i : ArrowPeaks){
        cv::circle(OriginalImage,i,1,cv::Scalar(153,156,30),-1);
        // std::stringstream ss;ss<<PeaksCnt<<":"<<(i-cv::Point(center)).cross(Centerline);PeaksCnt++;
        // cv::putText(OriginalImage,ss.str(),i,cv::FONT_HERSHEY_SIMPLEX,1.0,cv::Scalar(225,225,225));
    }

    DrawLines(OriginalImage,Lines{ArrowLines[DistanceBTOutCornerLines[0].first],ArrowLines[DistanceBTOutCornerLines[1].first]},cv::Scalar(66,66,225));
    DrawLines(OriginalImage,Lines{ArrowLines[DistanceBTInCornerLines[0].first],ArrowLines[DistanceBTInCornerLines[1].first]},cv::Scalar(225,66,66));

    cv::imshow("HoughLines",OriginalImage);
    cv::waitKey(33);

    #endif

    this->ArrowPeaks.clear();
    for(int i=0;i<4;i++){
        this->ArrowPeaks.push_back(cv::Point2d(ArrowPeaks[i].x,ArrowPeaks[i].y));
    }
    RCLCPP_INFO(this->get_logger(),"TargetArrow succesfully");
    return 1;
}

bool Arrow_detector::MainDetectArrow(const cv::Mat & OriginalImage){
    cv::Mat Binary=PreProgress(OriginalImage);

    bool HaveArrow=TargetArrow(Binary);
    if(!HaveArrow){
        RCLCPP_INFO(this->get_logger(),"fail to target arrow.");
        return 0;
    }

    PnPsolver();

    return 1;
}

cv::Mat Arrow_detector::PreProgress(const cv::Mat & OriginalImage){
    std::vector<cv::Mat> SplitImage;
    //通道顺序为BGR
    cv::split(OriginalImage,SplitImage);
    
    cv::Mat GreyImage(SplitImage[0].size(),SplitImage[0].type()),BinaryImage,DilatedImage;
    
    cv::mixChannels(std::vector<cv::Mat>{SplitImage[0],SplitImage[2]},
        std::vector<cv::Mat>{GreyImage},
        std::vector<int>{
            0,0,
            1,0
    });

    double maxval,minval;

    //控制二值化的参数
    double threshholdk=0.5;
    cv::minMaxLoc(GreyImage,&minval,&maxval);
    minval=std::max(150.0,minval);
    maxval=std::max(minval,maxval);

    RCLCPP_INFO(this->get_logger(),"maxval of greyimage : %lf ,minval of greyimage : %lf ",maxval,minval);

    cv::threshold(GreyImage,BinaryImage,100,300,cv::THRESH_BINARY);

    cv::dilate(BinaryImage,DilatedImage,cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(3,3)));

    // cv::imshow("Pre",BinaryImage);
    // cv::imshow("Ori",OriginalImage);
    // cv::imshow("Gre",GreyImage);
    #ifdef DeBug
    cv::imshow("Dilated",DilatedImage);
    cv::waitKey(33);
    #endif

    return DilatedImage;
}

int main (int argc,char* argv[]){
    rclcpp::init(argc,argv);
    auto node=std::make_shared<Arrow_detector>();
    node->CreatGetImageTimer();
    // rclcpp::TimerBase::SharedPtr timer_=node->create_wall_timer(33ms,std::bind(&Arrow_detector::GetImage,node));

    // while(1){
    //     // std::this_thread::sleep_for(33ms);
    //     node->GetImage();
    // }

    rclcpp::spin(node);
    rclcpp::shutdown();
}