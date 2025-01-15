#include "DetectArrow.hpp"

cv::Mat Arrow_detector::OOriginalImage(1440,1080,CV_8UC3,cv::Scalar(0,0,0));

std::vector<double> cameraMatrix={2395.321201462218, 0, 694.4679804647494,
 0, 2395.631182994724, 565.4935222230949,
 0, 0, 1};

std::vector<double> distCoeffs={-0.05759433071291451, 0.0006891020898386943, -0.0004902067374258133, 7.184226741273259e-05, 2.151048297510786};

std::vector<cv::Point3d> objpoints={
    cv::Point3d(0,0,0),
    cv::Point3d(10,10,0),
    cv::Point3d(141.42135623730950488016887242097,0,0),
    cv::Point3d(131.42135623730950488016887242097,10,0),
    cv::Point3d(0,141.42135623730950488016887242097,0),
    cv::Point3d(10,131.42135623730950488016887242097,0),
    cv::Point3d(10,0,0),
    cv::Point3d(0,10,0)
};

std::vector<Eigen::Matrix<double,4,1>> objpointsEigen{
    Eigen::Matrix<double,4,1>(0,0,0,1),
    Eigen::Matrix<double,4,1>(10,10,0,1),
    Eigen::Matrix<double,4,1>(141.42135623730950488016887242097,0,0,1),
    Eigen::Matrix<double,4,1>(131.42135623730950488016887242097,10,0,1),
    Eigen::Matrix<double,4,1>(0,141.42135623730950488016887242097,0,1),
    Eigen::Matrix<double,4,1>(10,131.42135623730950488016887242097,0,1),
    Eigen::Matrix<double,4,1>(10,0,0,1),
    Eigen::Matrix<double,4,1>(0,10,0,1)
};

std::vector<cv::Point3d> ObjRedemptionBoxCornerPoint={
    cv::Point3d(-117.02617228637361528833974192835,52.679455198397790567862904976811,24),
    cv::Point3d(52.679455198397790567862904976811,-117.02617228637361528833974192835,24),
    cv::Point3d(-117.02617228637361528833974192835,52.679455198397790567862904976811,264),
    cv::Point3d(52.679455198397790567862904976811,-117.02617228637361528833974192835,264)
};

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

Eigen::Matrix<double,4,4> CenterToArrowvec;



int ArrowDetectorPixelNumMax;
int ArrowDetectorPixelNumMin;
double ArrowDetectorLengthWidthRatioMax;
double ArrowDetectorLengthWidthRatioMin;
int ArrowDetectorApproxSizeMax;
int ArrowDetectorApproxSizeMin;
double ArrowDetectorCannyThreshold1;
double ArrowDetectorCannyThreshold2;
double ArrowDetectorHoughRho;
double ArrowDetectorHoughTheta;
int ArrowDetectorHoughThreshold;
double ArrowDetectParallelThreshold;
double ArrowDetectorThresholdThresh;
double ArrowDetectorThresholdMaxval;
double ArrowDetectorThresholdThreshold;
int ArrowDetectorIterations;
double ArrowDetectorapproxPolyDPEpsilon;

// template<typename T,typename G>
bool Arrow_detector::PnPsolver(const std::vector<cv::Point2f > & ImagePoints2D,const std::vector<cv::Point3d > & ObjectPoints3D,const std::vector<double> & cameraMatrix,const std::vector<double> & distCoeffs,
    cv::Mat & rvec, cv::Mat & tvec, bool useExtrinsicGuess, int flags){
    cv::Mat cameraMatrixCV=cv::Mat(3,3,CV_64F,const_cast<double *>(cameraMatrix.data())).clone();
    cv::Mat distCoeffsCV=cv::Mat(1,5,CV_64F,const_cast<double *>(distCoeffs.data())).clone();
    Eigen::Matrix<double,3,3> cameraMatrixEigen;
    Eigen::Matrix<double,4,4> rtvecEigen;
    Eigen::Matrix<double,3,4> signMat;

    bool PnPsuccess=cv::solvePnP(ObjectPoints3D,ImagePoints2D,cameraMatrixCV,distCoeffsCV,rvec,tvec,useExtrinsicGuess,flags);

    if(!PnPsuccess){
        RCLCPP_WARN(this->get_logger(),"PnP fail");
        return 0;
    }

    std::stringstream ss;
    ss<<"cameraMatrixCV\n"<<cameraMatrixCV;
    ss<<"distCoeffsCV\n"<<distCoeffsCV;
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

    Counter corners;
    for(const auto & i : Object2cornersEigen){
        auto coordination=cameraMatrixEigen*signMat*rtvecEigen*i;
        corners.push_back(cv::Point2i(coordination(0)/coordination(2),coordination(1)/coordination(2)));
    }

    // #ifdef DeBug
    
    cv::drawContours(OriginalImage,Counters{corners},-1,cv::Scalar(225,0,0),5);

    cv::drawContours(OriginalImage,Counters{ImageRedemptionBoxCornerPoints},-1,cv::Scalar(225,0,0),3);

    // cv::putText(OriginalImage,ss.str().c_str(),cv::Point(0,0),cv::FONT_HERSHEY_SIMPLEX,1.0,cv::Scalar(225,0,0));

    // #endif



    static int CntVideo=0;
    CntVideo++;
    videowriter<<OriginalImage;
    RCLCPP_INFO(this->get_logger(),"wirte video %d",CntVideo);

    Eigen::Matrix<double,4,1> corn={120,120,0,1};
    auto RedeemVec=rtvecEigen*CenterToArrowvec;
    auto Center3D=RedeemVec*corn;
    cv::Mat RedeemtVec33(cv::Size(3,3),CV_32F),RedeemtVec31(cv::Size(3,1),CV_32F);
    for(int i=0;i<3;i++) for(int e=0;e<3;e++) RedeemtVec33.at<float>(e,i)=RedeemVec(e,i);

    cv::Rodrigues(RedeemtVec33,RedeemtVec31);
    RCLCPP_INFO(this->get_logger(),"1");

    interfaces::msg::RedeemBoxPosition::SharedPtr msg=std::make_shared<interfaces::msg::RedeemBoxPosition>();

    RCLCPP_INFO(this->get_logger(),"2");


    msg->center.x=Center3D(0)/Center3D(3);
    msg->center.y=Center3D(1)/Center3D(3);
    msg->center.z=Center3D(2)/Center3D(3);

    RCLCPP_INFO(this->get_logger(),"3");

    msg->rvec=std::vector<double>{RedeemtVec31.at<float>(0),RedeemtVec31.at<float>(1),RedeemtVec31.at<float>(2)};
    msg->tvec=std::vector<double>{RedeemVec(0,3),RedeemVec(1,3),RedeemVec(2,3)};
    publisher_->publish(*msg);
    RCLCPP_INFO(this->get_logger(),"Publish RedemptionBox Pos");

    auto tempelat=(ObjRedemptionBoxCornerPointEigen[0]+ObjRedemptionBoxCornerPointEigen[1]+ObjRedemptionBoxCornerPointEigen[2]+ObjRedemptionBoxCornerPointEigen[3])/4;
    auto Center=cameraMatrixEigen*signMat*rtvecEigen*tempelat;


    auto Center2=cameraMatrixEigen*signMat*RedeemVec*corn;
    RCLCPP_INFO(this->get_logger(),"%lf",RedeemVec(15));

    std::stringstream ssss;
    ssss<<"Center: "<<Center<<" Center2: "<<Center2;
    RCLCPP_INFO(this->get_logger(),"%s",ssss.str().c_str());

    cv::circle(OriginalImage,cv::Point(Center2(0)/Center2(2),Center2(1)/Center2(2)),1,cv::Scalar(223,225,133),-1);
    cv::putText(OriginalImage,"1",cv::Point(Center2(0)/Center2(2),Center2(1)/Center2(2)),cv::FONT_HERSHEY_SIMPLEX,1.0,cv::Scalar(225,225,225));
    cv::circle(OriginalImage,cv::Point(Center(0)/Center(2),Center(1)/Center(2)),1,cv::Scalar(223,225,133),-1);
    cv::putText(OriginalImage,"2",cv::Point(Center(0)/Center(2),Center(1)/Center(2)),cv::FONT_HERSHEY_SIMPLEX,1.0,cv::Scalar(225,225,225));
    

    cv::imshow("PnP",OriginalImage);
    cv::waitKey(0);

    RCLCPP_INFO(this->get_logger(),"PnPsolver finish");

    return 1;
}

void Arrow_detector::InitialArrowDetector(){
    using namespace std::placeholders;
    using namespace std::chrono;
    node_shred_ptr=this->shared_from_this();

    CenterToArrowvec<<0.70710678118654752440084436210485,0., 0.70710678118654752440084436210485, -31.466251762801364835837574113666,
    -0.70710678118654752440084436210485,0.0, 0.70710678118654752440084436210485, -31.466251762801364835837574113666,
    0., 1., 0.0, 144.,
    0., 0., 0., 1.;

    this->declare_parameter<int>("ArrowDetectorPixelNumMax",80000);
    this->declare_parameter<int>("ArrowDetectorPixelNumMin",15000);
    this->declare_parameter<double>("ArrowDetectorLengthWidthRatioMax",4/sqrt(2));
    this->declare_parameter<double>("ArrowDetectorLengthWidthRatioMin",1.1);
    this->declare_parameter<int>("ArrowDetectorApproxSizeMax",10);
    this->declare_parameter<int>("ArrowDetectorApproxSizeMin",6);

    this->declare_parameter<double>("ArrowDetectorCannyThreshold1",100);
    this->declare_parameter<double>("ArrowDetectorCannyThreshold2",200);
    this->declare_parameter<double>("ArrowDetectorHoughRho",0.25);
    this->declare_parameter<double>("ArrowDetectorHoughTheta",CV_PI/720);
    this->declare_parameter<int>("ArrowDetectorHoughThreshold",35);
    this->declare_parameter<double>("ArrowDetectParallelThreshold",CV_PI/6);

    this->declare_parameter<double>("ArrowDetectorThresholdThresh",120);
    this->declare_parameter<double>("ArrowDetectorThresholdMaxval",300);

    this->declare_parameter<double>("ArrowDetectorThresholdThreshold",5);

    this->declare_parameter<int>("ArrowDetectorIterations",5);

    this->declare_parameter<double>("ArrowDetectorapproxPolyDPEpsilon",20);

    ArrowDetectorPixelNumMax=this->get_parameter("ArrowDetectorPixelNumMax").as_int();
    ArrowDetectorPixelNumMin=this->get_parameter("ArrowDetectorPixelNumMin").as_int();
    ArrowDetectorLengthWidthRatioMax=this->get_parameter("ArrowDetectorLengthWidthRatioMax").as_double();
    ArrowDetectorLengthWidthRatioMin=this->get_parameter("ArrowDetectorLengthWidthRatioMin").as_double();
    ArrowDetectorApproxSizeMax=this->get_parameter("ArrowDetectorApproxSizeMax").as_int();
    ArrowDetectorApproxSizeMin=this->get_parameter("ArrowDetectorApproxSizeMin").as_int();

    ArrowDetectorCannyThreshold1=this->get_parameter("ArrowDetectorCannyThreshold1").as_double();
    ArrowDetectorCannyThreshold2=this->get_parameter("ArrowDetectorCannyThreshold2").as_double();
    ArrowDetectorHoughRho=this->get_parameter("ArrowDetectorHoughRho").as_double();
    ArrowDetectorHoughTheta=this->get_parameter("ArrowDetectorHoughTheta").as_double();
    ArrowDetectorHoughThreshold=this->get_parameter("ArrowDetectorHoughThreshold").as_int();
    ArrowDetectParallelThreshold=this->get_parameter("ArrowDetectParallelThreshold").as_double();

    ArrowDetectorThresholdThresh=this->get_parameter("ArrowDetectorThresholdThresh").as_double();
    ArrowDetectorThresholdMaxval=this->get_parameter("ArrowDetectorThresholdMaxval").as_double();

    ArrowDetectorThresholdThreshold=this->get_parameter("ArrowDetectorThresholdThreshold").as_double();

    ArrowDetectorIterations=this->get_parameter("ArrowDetectorIterations").as_int();
    ArrowDetectorapproxPolyDPEpsilon=this->get_parameter("ArrowDetectorapproxPolyDPEpsilon").as_double();
}

void Arrow_detector::GetImage(const sensor_msgs::msg::Image::SharedPtr msg){
    if(!rclcpp::ok()){
        rclcpp::shutdown();
    }
    // if((this->now()-msg->header.stamp)>=rclcpp::Duration(1,5000'000000)){
        //超过10ms丢弃
        // RCLCPP_INFO(this->get_logger(),"Time out.");
        // return;
    // }
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
    originalframe.copyTo(Arrow_detector::OOriginalImage);
    RCLCPP_INFO(this->get_logger(), "Get frame");
    MainDetectArrow(originalframe);

}

bool Arrow_detector::TargetArrow(const cv::Mat & BinaryImage){
    std::vector<cv::Vec2f> lines;
    Counters counters_;
    Counter isarrow;
    Counter arrowapproxcurve;
    
    cv::findContours(BinaryImage,counters_,cv::RETR_LIST,cv::CHAIN_APPROX_SIMPLE);
    // cv::drawContours(OriginalImage,counters_,-1,cv::Scalar(224,33,21),1);


    cv::Point2f center;
    std::vector<cv::Point2f> TrianglePeaks;
    float radius;
    cv::Mat MaskedImage;
    std::vector<Slope> slopes;
    std::vector<std::pair<Slope,Slope>> HorizonLinePair;
    std::vector<bool> UsedLineMakePair(20,false);
    double HorizonThreshold=5,RThreshold=10,LThreshold=0;
    int NowMaxSize=0;

    for(auto &counter_ :counters_){

        cv::RotatedRect rotatedrect_=cv::minAreaRect(counter_);
        Counter approxcurve;

        double LengthWidthRatio= (std::min(rotatedrect_.size.width,rotatedrect_.size.height)<=eps ? 
            -1 : std::max(rotatedrect_.size.width,rotatedrect_.size.height)/std::min(rotatedrect_.size.width,rotatedrect_.size.height));
        int pixel_num=cv::contourArea(counter_);
        cv::approxPolyDP(counter_,approxcurve,ArrowDetectorapproxPolyDPEpsilon,1);

        // cv::putText(OriginalImage,std::to_string(approxcurve.size()),rotatedrect_.center,cv::FONT_HERSHEY_COMPLEX,1.0,cv::Scalar(223,123,43));

        RCLCPP_INFO(this->get_logger(),"LengthWidthRatio : %lf",LengthWidthRatio);
        RCLCPP_INFO(this->get_logger(),"pixel_num : %d",pixel_num);
        RCLCPP_INFO(this->get_logger(),"approxcurve size : %ld",approxcurve.size());

        bool pixel_in=(pixel_num>=ArrowDetectorPixelNumMin&&
            pixel_num<=ArrowDetectorPixelNumMax);
        bool lwratio=(ArrowDetectorLengthWidthRatioMin<=LengthWidthRatio&&
            LengthWidthRatio<=ArrowDetectorLengthWidthRatioMax);
        bool approxsize=(std::size_t(ArrowDetectorApproxSizeMin)<=approxcurve.size()&&
            approxcurve.size()<=std::size_t(ArrowDetectorApproxSizeMax));

        if(!(pixel_in&&lwratio&&approxsize&&(NowMaxSize<pixel_num))) continue;
        cv::minEnclosingCircle(counter_,center,radius);
        cv::minEnclosingTriangle(counter_,TrianglePeaks);

        slopes.clear();
        for(int i=approxcurve.size()-1,siz=approxcurve.size();i>=0;i--){
            slopes.push_back(Slope{i,(i+1)%siz,[](cv::Point p1,cv::Point p2){
                double angle=GetAngleAccordingToHorizon(p1,p2);
                return abs(angle-180)<5 ? 0 : angle;}(approxcurve[i],approxcurve[(i+1)%siz])});
        }
        std::sort(slopes.begin(),slopes.end(),[](const Slope & a,const Slope & b){
            return a.slope<b.slope;
        });
        
        int TryCnt=0;
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
            continue;
        }
        else RCLCPP_INFO(this->get_logger(),"find horizon line pairs!");


        NowMaxSize=pixel_num;
        isarrow=std::move(counter_);
        arrowapproxcurve=std::move(approxcurve);
        cv::drawContours(OriginalImage,Counters{isarrow},-1,cv::Scalar(225,225,225),1);
        cv::drawContours(OriginalImage,Counters{arrowapproxcurve},-1,cv::Scalar(0,225,225),1);
        // cv::imshow("DETECT GET",OriginalImage);
        // cv::waitKey(0);

    }

    #ifdef DeBug

    cv::imshow("Counters",OriginalImage);
    cv::waitKey(33);

    #endif

    if(isarrow.empty()){
        RCLCPP_WARN(this->get_logger(),"fail to find arrow!");
        return 0;
    }
    else RCLCPP_INFO(this->get_logger(),"find arrow!");

    cv::Mat Mask(OriginalImage.size(),CV_8UC1,cv::Scalar(0));
    // cv::minEnclosingCircle(isarrow,center,radius);
    // cv::minEnclosingTriangle(isarrow,TrianglePeaks);

    cv::circle(Mask,center,radius,cv::Scalar(255),-1);

    cv::copyTo(BinaryImage,MaskedImage,Mask);

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
        return 0;
    }
    else RCLCPP_INFO(this->get_logger(),"finish dichotomy and find two right angle peaks");

    if(DistancePoints(center,arrowapproxcurve[RightAnglePeaks[0]])<DistancePoints(center,arrowapproxcurve[RightAnglePeaks[1]])){
        std::swap(RightAnglePeaks[0],RightAnglePeaks[1]);
    }
    //确定第一个为外侧点，第二个为内侧点

    RCLCPP_INFO(this->get_logger(),"find right angle !");
    /*
    第一次迭代顶点储存
    储存规则：
    最外侧直角顶点，最内侧直角顶点，从中心线外接圆顺时针方向第一个尾处的两顶点(外侧在前)，从中心线外接圆顺时针方向第二尾处的两顶点(外侧在前)
    */
    std::vector<cv::Point> ArrowPeaks;

    ArrowPeaks.push_back(arrowapproxcurve[RightAnglePeaks[0]]);
    ArrowPeaks.push_back(arrowapproxcurve[RightAnglePeaks[1]]);
    ArrowPeaks.push_back(cv::Point(0,0));
    ArrowPeaks.push_back(cv::Point(0,0));
    ArrowPeaks.push_back(cv::Point(0,0));
    ArrowPeaks.push_back(cv::Point(0,0));

    cv::Point Centerline=ArrowPeaks[0]-cv::Point(center.x,center.y);
    for(int i=0;i<=1;i++){
        std::pair<int,int> PointNumPair1=std::make_pair(HorizonLinePair[i].first.p1,HorizonLinePair[i].first.p2);
        std::pair<int,int> PointNumPair2=std::make_pair(HorizonLinePair[i].second.p1,HorizonLinePair[i].second.p2);
        if(PointNumPair1.first==RightAnglePeaks[0]||PointNumPair1.first==RightAnglePeaks[1]) std::swap(PointNumPair1.first,PointNumPair1.second);
        if(PointNumPair2.first==RightAnglePeaks[0]||PointNumPair2.first==RightAnglePeaks[1]) std::swap(PointNumPair2.first,PointNumPair2.second);
        cv::Point TargetLine=arrowapproxcurve[PointNumPair1.first]-cv::Point(center.x,center.y);
        if(TargetLine.cross(Centerline)<=0){
            if(DistancePoints(arrowapproxcurve[PointNumPair1.first],center)>DistancePoints(arrowapproxcurve[PointNumPair2.first],center)){
                ArrowPeaks[2]=(arrowapproxcurve[PointNumPair1.first]);
                ArrowPeaks[3]=(arrowapproxcurve[PointNumPair2.first]);
            }
            else{
                ArrowPeaks[2]=(arrowapproxcurve[PointNumPair2.first]);
                ArrowPeaks[3]=(arrowapproxcurve[PointNumPair1.first]);
            }
        }
        else{
            if(DistancePoints(arrowapproxcurve[PointNumPair1.first],center)>DistancePoints(arrowapproxcurve[PointNumPair2.first],center)){
                ArrowPeaks[4]=(arrowapproxcurve[PointNumPair1.first]);
                ArrowPeaks[5]=(arrowapproxcurve[PointNumPair2.first]);
            }
            else{
                ArrowPeaks[4]=(arrowapproxcurve[PointNumPair2.first]);
                ArrowPeaks[5]=(arrowapproxcurve[PointNumPair1.first]);
            }
        }
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


    // cv::imshow("ArrowPeaks",OriginalImage);
    // cv::waitKey(22);

    #endif

    cv::Mat CannyImage;
    cv::Canny(MaskedImage,CannyImage,ArrowDetectorCannyThreshold1,ArrowDetectorCannyThreshold2);

    std::vector<std::vector<cv::Point>> LinesPoints;
    std::vector<LineVP> FittedLines;
    std::vector<std::pair<cv::Point,cv::Point> > Endpoints;
    std::vector<std::pair<int,int>> EndpointsIndex;

    // #ifdef DeBugHough

    // cv::imshow("CannyImage",CannyImage);
    // cv::waitKey(33);

    // #endif

    FindPolygonCounterPointsSets(CannyImage,LinesPoints,
        ArrowPeaks,
        ArrowDetectorThresholdThreshold,
        Endpoints);

    for(auto &p : Endpoints){
        std::pair<int,int> index=std::make_pair(-1,-1);
        for(int i=0;i<6;i++){
            if(p.first==ArrowPeaks[i]) index.first=i;
            if(p.second==ArrowPeaks[i]) index.second=i;
        }
        if(index.first==-1||index.second==-1){
            RCLCPP_ERROR(this->get_logger(),"ArrowPeaks mismatch");
            rclcpp::shutdown();
        }
        if(index.first>index.second) std::swap(index.first,index.second);
        EndpointsIndex.push_back(index);
    }

    if(Endpoints.size()!=6){
        RCLCPP_WARN(this->get_logger(),"size of Endpoints : %ld which is not equal to 6",Endpoints.size());
        return 0;
    }
    else RCLCPP_INFO(this->get_logger(),"size of Endpoints : %ld",Endpoints.size());

    if(EndpointsIndex.size()!=6){
        RCLCPP_WARN(this->get_logger(),"size of Endpoints : %ld which is not equal to 6",EndpointsIndex.size());
        return 0;
    }
    else RCLCPP_INFO(this->get_logger(),"size of Endpoints : %ld",EndpointsIndex.size());


    for(std::size_t i=0;i<LinesPoints.size();i++){
        cv::Vec4d line;
        cv::fitLine(LinesPoints[i],line,cv::DIST_L2,0,0.01,0.01);
        FittedLines.push_back(line);
        RCLCPP_INFO(this->get_logger(),"Line Info : %lf %lf %lf %lf",line[0],line[1],line[2],line[3]);
    }

    if(FittedLines.size()!=6){
        RCLCPP_WARN(this->get_logger(),"size of FittedLines : %ld which is not equal to 6",FittedLines.size());
        return 0;
    }
    else RCLCPP_INFO(this->get_logger(),"size of FittedLines : %ld",FittedLines.size());
    RCLCPP_INFO(this->get_logger(),"finish find lines");


    DrawLines(OriginalImage,FittedLines,cv::Scalar(225,225,225),1);


    #ifdef DeBugHough

    DrawLines(CannyImage,FittedLines,cv::Scalar(225),1);

    cv::imshow("FittedLines",OriginalImage);
    cv::imshow("CannyImageDD",CannyImage);

    #endif

    //第二次迭代顶点储存
    std::vector<cv::Point2f> ArrowPeaks_;
    
    static std::vector<std::pair<std::pair<int,int>,std::pair<int,int> > > MapOfIntersectionsLines={
        std::make_pair(std::make_pair(0,4),std::make_pair(0,2)),
        std::make_pair(std::make_pair(1,5),std::make_pair(1,3)),
        std::make_pair(std::make_pair(2,3),std::make_pair(0,2)),
        std::make_pair(std::make_pair(2,3),std::make_pair(1,3)),
        std::make_pair(std::make_pair(0,4),std::make_pair(4,5)),
        std::make_pair(std::make_pair(4,5),std::make_pair(1,5)),
        std::make_pair(std::make_pair(0,2),std::make_pair(1,5)),
        std::make_pair(std::make_pair(0,4),std::make_pair(1,3)),
    };

    for(int i=0;i<8;i++){
        int index1=-1,index2=-1;
        for(int e=0;e<6;e++){
            if(MapOfIntersectionsLines[i].first==EndpointsIndex[e]) index1=e;
            if(MapOfIntersectionsLines[i].second==EndpointsIndex[e]) index2=e;
        }
        ArrowPeaks_.push_back(GetLineIntersections(FittedLines[index1],FittedLines[index2]));
    }

    #ifdef DeBug

    for(int i=0;i<8;i++){
        // OriginalImage.at<cv::Vec3f>(int(ArrowPeaks_[i].y),int(ArrowPeaks_[i].x))=cv::Vec3f(0,0,0);
        cv::circle(OriginalImage,ArrowPeaks_[i],1,cv::Scalar(32,122,225),-1);
        cv::putText(OriginalImage,std::to_string(i),ArrowPeaks_[i],cv::FONT_HERSHEY_COMPLEX,1.0,cv::Scalar(32,122,225));
        RCLCPP_INFO(this->get_logger(),"Point %d: [%f,%f]",i,ArrowPeaks_[i].x,ArrowPeaks_[i].y);
    }
    #endif

    if(ArrowPeaks_.size()!=8){
        RCLCPP_ERROR(this->get_logger(),"Fail to find all peaks, size of ArrowPeaks_ : %ld",ArrowPeaks_.size());
        return 0;
    }
    else RCLCPP_INFO(this->get_logger(),"find all peaks successfully");

    // filter_.Update(ArrowPeaks_);

    this->ArrowPeaks=ArrowPeaks_;
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

    PnPsolver(ArrowPeaks,objpoints,cameraMatrix,distCoeffs,rvec,tvec,0,cv::SOLVEPNP_ITERATIVE);

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
    cv::threshold(GreyImage,BinaryImage,ArrowDetectorThresholdThresh,ArrowDetectorThresholdMaxval,cv::THRESH_BINARY);

    cv::erode(BinaryImage,DilatedImage,cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(3,3)),cv::Point(-1,-1),ArrowDetectorIterations);

    cv::Mat sharpening_kenel=(cv::Mat_<float>(3,3)<<
        0,-1,0,
        -1,5,-1,
        0,-1,0
    );
    cv::Mat Sharpened;
    cv::filter2D(GreyImage,Sharpened,-1,sharpening_kenel);

    cv::Mat blurred;
    cv::GaussianBlur(GreyImage,blurred,cv::Size(7,7),0);
    cv::Mat high=GreyImage-blurred;
    cv::Mat enhance=Sharpened+1.5*high;

    cv::Mat Binary_en;
    cv::threshold(enhance,Binary_en,ArrowDetectorThresholdThresh,ArrowDetectorThresholdMaxval,cv::THRESH_BINARY);

    cv::imshow("Sharpened",Sharpened);
    cv::imshow("enhance",enhance);
    cv::imshow("Binary_en",Binary_en);

    cv::imshow("GreyImage",GreyImage);
    cv::imshow("BinaryImage",BinaryImage);
    cv::imshow("DilatedImage",DilatedImage);

    // return DilatedImage;
    // return BinaryImage;
    return Binary_en;
}

int main (int argc,char* argv[]){
    rclcpp::init(argc,argv);
    auto node=std::make_shared<Arrow_detector>(0.9);
    node->InitialArrowDetector();

    rclcpp::spin(node);
    rclcpp::shutdown();
}