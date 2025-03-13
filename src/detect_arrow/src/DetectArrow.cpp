#include "DetectArrow.hpp"

bool Arrow_detector::PnPsolver(const std::vector<cv::Point2d > & ImagePoints2D,const std::vector<cv::Point3d > & ObjectPoints3D,const std::vector<double> & cameraMatrix,const std::vector<double> & distCoeffs,
    cv::Mat & rvec, cv::Mat & tvec, bool useExtrinsicGuess, int flags){
    cv::Mat cameraMatrixCV=cv::Mat(3,3,CV_64F,const_cast<double *>(cameraMatrix.data())).clone();
    cv::Mat distCoeffsCV=cv::Mat(1,5,CV_64F,const_cast<double *>(distCoeffs.data())).clone();
    
    //pnp homogeneous transfomration matrix
    Eigen::Matrix<double,4,4> rtvecEigen;

    std::stringstream ss_;

    // ss_<<"cameraMatrixCV\n"<<cameraMatrixCV;
    // ss_<<"distCoeffsCV\n"<<distCoeffsCV;
    // RCLCPP_INFO(this->get_logger(),"%s",ss_.str().c_str());
    // RCLCPP_INFO(this->get_logger(),"%d",ObjectPoints3D.size());
    // RCLCPP_INFO(this->get_logger(),"%d",ImagePoints2D.size());

    RCLCPP_INFO(this->get_logger(),"start pnp");
    // bool PnPsuccess=cv::solvePnPRansac(
    //     ObjectPoints3D,
    //     ImagePoints2D,
    //     cameraMatrixCV,
    //     distCoeffsCV,
    //     rvec,
    //     tvec,
    //     false,
    //     200,
    //     8.0,
    //     0.99,
    //     cv::noArray(),
    //     cv::SOLVEPNP_EPNP
    // );
    bool PnPsuccess=cv::solvePnP(ObjectPoints3D,ImagePoints2D,cameraMatrixCV,distCoeffsCV,rvec,tvec,useExtrinsicGuess,flags);

    for(int i=0;i<3;i++){
        if(std::isnan(rvec.at<double>(i))||std::isnan(tvec.at<double>(i))){
            RCLCPP_WARN(this->get_logger(),"PNP get NAN!!!!!");
            return 0;
        }
    }

    if(!PnPsuccess){
        RCLCPP_WARN(this->get_logger(),"PnP fail");
        return 0;
    }
    else{
        RCLCPP_INFO(this->get_logger(),"PnP success");
    }

    std::stringstream ss;
    ss<<"cameraMatrixCV\n"<<cameraMatrixCV;
    ss<<"distCoeffsCV\n"<<distCoeffsCV;
    ss<<"rvec: "<<rvec.at<double>(0)<<" "<<rvec.at<double>(1)<<" "<<rvec.at<double>(2)<<" "<<std::endl;
    ss<<"tvec: "<<tvec.at<double>(0)<<" "<<tvec.at<double>(1)<<" "<<tvec.at<double>(2)<<" "<<std::endl;
    RCLCPP_INFO(this->get_logger(),"%s",ss.str().c_str());
    RCLCPP_INFO(this->get_logger(),"finish pnp");

    cv::Mat rmat(rvec.size(),rvec.type());
    cv::Rodrigues(rvec,rmat);

    #ifdef arrow_draw

    std::stringstream rvecss;
    rvecss<<rvec<<std::endl;
    cv::putText(OriginalImage_,rvecss.str().c_str(),cv::Point(10,100),cv::FONT_HERSHEY_SIMPLEX,1.0,cv::Scalar(0,0,225));

    #endif

    //fill homogeneous transfomration matrix
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
        Eigen::Matrix<double,3,1> coordination=cameraMatrixEigen*signMat*rtvecEigen*i;
        coordination/=coordination(2);
        ImageRedemptionBoxCornerPoints.push_back(cv::Point2i(coordination(0),coordination(1)));
        
        std::stringstream ss;
        ss<<cameraMatrixEigen<<"\n"<<signMat<<"\n"<<rtvecEigen<<"\n"<<i<<"\n"<<coordination;
        RCLCPP_INFO(this->get_logger(),"Node : %s",ss.str().c_str());

    }

    Counter corners;
    for(const auto & i : Object2cornersEigen){
        Eigen::Matrix<double,3,1> coordination=cameraMatrixEigen*signMat*rtvecEigen*i;
        coordination/=coordination(2);
        corners.push_back(cv::Point2i(coordination(0),coordination(1)));
    }

    #ifdef arrow_draw
    
    cv::drawContours(OriginalImage_,Counters{corners},-1,cv::Scalar(225,0,0),5);

    cv::drawContours(OriginalImage_,Counters{ImageRedemptionBoxCornerPoints},-1,cv::Scalar(225,0,0),3);

    cv::putText(OriginalImage_,ss.str().c_str(),cv::Point(0,0),cv::FONT_HERSHEY_SIMPLEX,1.0,cv::Scalar(225,0,0));
    #endif

    //fill msg

    Eigen::Matrix<double,4,1> Center3D=rtvecEigen*frontfacecenter;
    Eigen::Matrix<double,4,1> CenterVectorz3D=rtvecEigen*Eigen::Matrix<double,4,1>(0,0,-1,1);
    Center3D/=Center3D(3);
    CenterVectorz3D/=CenterVectorz3D(3);


    interfaces::msg::RedeemBoxPosition::SharedPtr msg=std::make_shared<interfaces::msg::RedeemBoxPosition>();

    msg->center.x=Center3D(0);
    msg->center.y=Center3D(1);
    msg->center.z=Center3D(2);

    for(int i=0;i<4;i++) for(int e=0;e<4;e++) msg->homogeneous_transformation_matrix.push_back(rtvecEigen(i,e));

    msg->rvec=rvec;
    msg->tvec=tvec;

    msg->header.frame_id="/sensor/camera";
    msg->header.stamp=this->now();

    RedeemBoxPosition_publisher_->publish(*msg);

    geometry_msgs::msg::TransformStamped box_to_camera;

    cv::Vec4d Quaternion_r=rotationMatrixToQuaternion(rmat);

    box_to_camera.header.stamp=this->now();
    box_to_camera.header.frame_id="sensor/camera";
    box_to_camera.child_frame_id="object/box";
    box_to_camera.transform.translation.x=tvec.at<double>(0);
    box_to_camera.transform.translation.y=tvec.at<double>(1);
    box_to_camera.transform.translation.z=tvec.at<double>(2);
    box_to_camera.transform.rotation.w=Quaternion_r[0];
    box_to_camera.transform.rotation.x=Quaternion_r[1];
    box_to_camera.transform.rotation.y=Quaternion_r[2];
    box_to_camera.transform.rotation.z=Quaternion_r[3];

    tf_broadcaster_box_to_camera->sendTransform(box_to_camera);

    # ifdef arrow_draw

    std::vector<cv::Point> reput_arrow;

    std::vector<int> index={0,2,3,1,5,4};

    for(auto i : index){
        Eigen::Matrix<double,3,1> new_i=cameraMatrixEigen*signMat*rtvecEigen*objpointsEigen[i];
        new_i(0)/=new_i(2);
        reput_arrow.push_back(cv::Point(new_i(0),new_i(1)));
        RCLCPP_INFO(this->get_logger(),"reput_arrow %ld %ld",reput_arrow.back().x,reput_arrow.back().y);
    }

    // cv::drawContours(OriginalImage_,Counters{reput_arrow},-1,cv::Scalar(225,0,0),3);
    Eigen::Matrix<double,3,1> Center2D=cameraMatrixEigen*signMat*Center3D;
    Eigen::Matrix<double,3,1> CenterVectorz2D=cameraMatrixEigen*signMat*CenterVectorz3D;
    Center2D/=Center2D(2);
    CenterVectorz2D/=CenterVectorz2D(2);

    cv::line(OriginalImage_,cv::Point(Center2D(0),Center2D(1)),cv::Point(CenterVectorz2D(0),CenterVectorz2D(1)),cv::Scalar(225,200,100));

    cv::circle(OriginalImage_,cv::Point(Center2D(0),Center2D(1)),1,cv::Scalar(223,225,133),-1);
    cv::putText(OriginalImage_,"center",cv::Point(Center2D(0),Center2D(1)),cv::FONT_HERSHEY_SIMPLEX,1.0,cv::Scalar(225,225,225));
    
    auto lable_msg_ptr=cv_bridge::CvImage(std_msgs::msg::Header(),sensor_msgs::image_encodings::BGR8,OriginalImage_).toImageMsg();
    lable_msg_ptr->header.frame_id="/arrow_detect/label_image";
    lable_msg_ptr->header.stamp=this->get_clock()->now();
    this->label_image_pub_->publish(*lable_msg_ptr);

    cv::imshow("pnp result",OriginalImage_);

    cv::waitKey(22);

    # endif

    RCLCPP_INFO(this->get_logger(),"PnPsolver finish");
    return 1;
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
    originalframe.copyTo(OriginalImage_);
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

    RCLCPP_INFO(this->get_logger(),"%ld",counters_.size());
    for(auto &counter_ :counters_){
        cv::RotatedRect rotatedrect_;
        try{
            RCLCPP_INFO(this->get_logger(),"%ld",counter_.size());
            rotatedrect_=cv::minAreaRect(counter_);
            RCLCPP_INFO(this->get_logger(),"%ld",counter_.size());
        }
        catch (cv::Exception & e){
            RCLCPP_WARN(this->get_logger(),"fail to find circle by minEnclosingCircle : %s",e.what());
            continue;
        }
        Counter approxcurve;

        double LengthWidthRatio= (std::min(rotatedrect_.size.width,rotatedrect_.size.height)<=eps ? 
            -1 : std::max(rotatedrect_.size.width,rotatedrect_.size.height)/std::min(rotatedrect_.size.width,rotatedrect_.size.height));
        int pixel_num=cv::contourArea(counter_);
        cv::approxPolyDP(counter_,approxcurve,ArrowDetectorapproxPolyDPEpsilon,1);

        bool pixel_in=(pixel_num>=ArrowDetectorPixelNumMin&&
            pixel_num<=ArrowDetectorPixelNumMax);
        bool lwratio=(ArrowDetectorLengthWidthRatioMin<=LengthWidthRatio&&
            LengthWidthRatio<=ArrowDetectorLengthWidthRatioMax);
        bool approxsize=(std::size_t(ArrowDetectorApproxSizeMin)<=approxcurve.size()&&
            approxcurve.size()<=std::size_t(ArrowDetectorApproxSizeMax));


        // if(approxcurve.size()<500) continue;

        #ifdef TargetArrowtest

        cv::Mat copy_;
        OriginalImage_.copyTo(copy_);

        cv::Point2f rotatedrect_points_ptr[4];
        std::vector<cv::Point2f> rotatedrect_points;
        rotatedrect_.points(rotatedrect_points_ptr);
        for(int i=0;i<4;i++){
            rotatedrect_points.push_back(rotatedrect_points_ptr[i]);
        }

        RCLCPP_INFO(this->get_logger(),"LengthWidthRatio : %lf",LengthWidthRatio);
        RCLCPP_INFO(this->get_logger(),"pixel_num : %d",pixel_num);
        RCLCPP_INFO(this->get_logger(),"approxcurve size : %ld",approxcurve.size());

        // std::stringstream center_ss;
        // center_ss<<center<<" "<<radius;
        // RCLCPP_INFO(this->get_logger(),"center_ss: %s",center_ss.str().c_str());

        RCLCPP_INFO(this->get_logger(),((pixel_in&&lwratio&&approxsize&&(NowMaxSize<pixel_num)) ? "pass" : "not pass"));

        std::stringstream rotate_ss;
        for(int i=0;i<4;i++){
            rotate_ss<<rotatedrect_points[i]<<" ";
        }
        RCLCPP_INFO(this->get_logger(),"rotate_ss: \n%s",rotate_ss.str().c_str());

        cv::drawContours(copy_,
            std::vector<std::vector<cv::Point>>{{cv::Point(rotatedrect_points[0].x,rotatedrect_points[0].y),cv::Point(rotatedrect_points[1].x,rotatedrect_points[1].y),cv::Point(rotatedrect_points[2].x,rotatedrect_points[2].y),cv::Point(rotatedrect_points[3].x,rotatedrect_points[3].y)}},
            -1,
            cv::Scalar(225,0,0),2);
        cv::drawContours(copy_,Counters{counter_},-1,cv::Scalar(225,0,0),1);

        cv::imshow("TargetArrow rotatedrect_", copy_);
        RCLCPP_INFO(this->get_logger(),"rotate_ss: \n%s",rotate_ss.str().c_str());
        cv::waitKey(0);

        #endif

        if(!(pixel_in&&lwratio&&approxsize&&(NowMaxSize<pixel_num))) continue;

        try{
            std::vector<cv::Point2f> lin;
            for(auto & i : counter_){
                lin.push_back(cv::Point2f(i.x,i.y));
            }
            cv::minEnclosingCircle(lin,center,radius);
            cv::minEnclosingTriangle(lin,TrianglePeaks);
        }
        catch (cv::Exception & e){
            RCLCPP_WARN(this->get_logger(),"fail to find circle by minEnclosingCircle : %s",e.what());
            continue;
        }
        RCLCPP_INFO(this->get_logger(),"narrow down to arrow by circle and triangle");


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

        # ifdef arrow_draw
        
        cv::drawContours(OriginalImage_,Counters{isarrow},-1,cv::Scalar(225,225,225),1);
        cv::drawContours(OriginalImage_,Counters{arrowapproxcurve},-1,cv::Scalar(0,225,225),1);

        # endif

    }

    if(isarrow.empty()){
        RCLCPP_WARN(this->get_logger(),"fail to find arrow!");
        return 0;
    }
    else RCLCPP_INFO(this->get_logger(),"find arrow!");

    cv::Mat Mask(BinaryImage.size(),CV_8UC1,cv::Scalar(0));

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
    std::vector<cv::Point2d> subArrowPeaks;

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

    # ifdef arrow_draw
    int PeaksCnt=0;
    for(auto i : ArrowPeaks){
        cv::circle(OriginalImage_,i,1,cv::Scalar(153,156,30),-1);
        std::stringstream ss;ss<<PeaksCnt<<":"<<(i-cv::Point(center)).cross(Centerline);PeaksCnt++;
        cv::putText(OriginalImage_,ss.str(),i,cv::FONT_HERSHEY_SIMPLEX,1.0,cv::Scalar(225,225,225));
    }

    #endif

    cv::Mat CannyImage;
    cv::Canny(MaskedImage,CannyImage,ArrowDetectorCannyThreshold1,ArrowDetectorCannyThreshold2);

    std::vector<std::vector<cv::Point>> LinesPoints;
    std::vector<LineVP> FittedLines;
    std::vector<std::pair<cv::Point2d,cv::Point2d> > Endpoints;
    std::vector<std::pair<int,int>> EndpointsIndex;

    #ifdef Imageshow

    cv::imshow("CannyImage",CannyImage);
    cv::waitKey(33);

    #endif

    // begine SubPix
    subArrowPeaks=subopix(this->GreyImage,
        ArrowPeaks,
        cv::Size(10,10),
        cv::Size(-1,-1),
        cv::TermCriteria(
            cv::TermCriteria::EPS+cv::TermCriteria::COUNT,
            150,
            0.001
            ),
        Mask);
    

    FindPolygonCounterPointsSets(CannyImage,LinesPoints,
        subArrowPeaks,
        ArrowDetectorThresholdThreshold,
        Endpoints);

    for(auto &p : Endpoints){
        std::pair<int,int> index=std::make_pair(-1,-1);
        for(int i=0;i<6;i++){
            if(IsPointSame(p.first,subArrowPeaks[i])) index.first=i;
            if(IsPointSame(p.second,subArrowPeaks[i])) index.second=i;
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

    # ifdef arrow_draw

    DrawLines(OriginalImage_,FittedLines,cv::Scalar(225,225,225),1);

    # endif

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

    #ifdef twopath_inoneline

    std::vector<cv::Point> index1,index2;
    int jjj=0;

    for(auto &p : Endpoints){

        if(( (p.first.x==ArrowPeaks[2].x && p.first.y==ArrowPeaks[2].y) && (p.second.x==ArrowPeaks[3].x && p.second.y==ArrowPeaks[3].y))||
        ( (p.first.x==ArrowPeaks[3].x && p.first.y==ArrowPeaks[3].y) && (p.second.x==ArrowPeaks[2].x && p.second.y==ArrowPeaks[2].y)) ){
            index1=LinesPoints[jjj];
        }
        if(( (p.first.x==ArrowPeaks[4].x && p.first.y==ArrowPeaks[4].y) && (p.second.x==ArrowPeaks[5].x && p.second.y==ArrowPeaks[5].y))||
        ( (p.first.x==ArrowPeaks[5].x && p.first.y==ArrowPeaks[5].y) && (p.second.x==ArrowPeaks[4].x && p.second.y==ArrowPeaks[4].y)) ){
            index2=LinesPoints[jjj];
        }
        jjj++;
    }

    if(index1.size()==0||index2.size()==0) return 0;

    std::vector<cv::Point> allindex=index1;

    for (auto i : index2) allindex.push_back(i);

    cv::Vec4d line_index;

    cv::fitLine(allindex,line_index,cv::DIST_L2,0,0.01,0.01);

    #ifdef arrow_draw
    DrawLines(OriginalImage_,std::vector<LineVP>{line_index},cv::Scalar(225,225,225),1);
    #endif

    std::vector<cv::Point2d> new_one(4);

    for(int i=0;i<6;i++){
        if(EndpointsIndex[i]==std::make_pair(0,4)) new_one[2]=GetLineIntersections(line_index,FittedLines[i]);
        if(EndpointsIndex[i]==std::make_pair(1,5)) new_one[3]=GetLineIntersections(line_index,FittedLines[i]);
        if(EndpointsIndex[i]==std::make_pair(1,3)) new_one[1]=GetLineIntersections(line_index,FittedLines[i]);
        if(EndpointsIndex[i]==std::make_pair(0,2)) new_one[0]=GetLineIntersections(line_index,FittedLines[i]);
    }
    for(int i=2;i<6;i++){
        ArrowPeaks_[i]=new_one[i-2];
    }
    #endif twopath_inoneline

    #ifdef arrow_draw

    for(int i=0;i<8;i++){
        // OriginalImage.at<cv::Vec3f>(int(ArrowPeaks_[i].y),int(ArrowPeaks_[i].x))=cv::Vec3f(0,0,0);
        cv::circle(OriginalImage_,ArrowPeaks_[i],1,cv::Scalar(32,122,225),-1);
        cv::putText(OriginalImage_,std::to_string(i),ArrowPeaks_[i],cv::FONT_HERSHEY_COMPLEX,1.0,cv::Scalar(32,122,225));
        RCLCPP_INFO(this->get_logger(),"Point %d: [%f,%f]",i,ArrowPeaks_[i].x,ArrowPeaks_[i].y);
    }
    #endif

    # ifndef arrow_draw

    for(int i=0;i<8;i++){
        RCLCPP_INFO(this->get_logger(),"Point %d: [%f,%f]",i,ArrowPeaks_[i].x,ArrowPeaks_[i].y);
    }
    # endif

    # ifdef Imageshow

    cv::imshow("finish one",OriginalImage_);
    cv::waitKey(33);

    #endif

    if(ArrowPeaks_.size()!=8){
        RCLCPP_ERROR(this->get_logger(),"Fail to find all peaks, size of ArrowPeaks_ : %ld",ArrowPeaks_.size());
        return 0;
    }
    else RCLCPP_INFO(this->get_logger(),"find all peaks successfully");

    // filter_.Update(ArrowPeaks_);
    this->ArrowPeaks.clear();
    for(auto & i : ArrowPeaks_) this->ArrowPeaks.push_back(i);
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

    PnPsolver(ArrowPeaks,objpoints,cameraMatrix,distCoeffs,rvec,tvec,0,cv::SOLVEPNP_IPPE);

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

    this->GreyImage=GreyImage;

    cv::Mat GaussBinaryImage;
    cv::GaussianBlur(GreyImage,
        GaussBinaryImage,
        cv::Size(5,5),
        1.5
    );

    cv::Mat sharpening_kenel=(cv::Mat_<float>(3,3)<<
        0,-1,0,
        -1,5,-1,
        0,-1,0
    );
    cv::Mat Sharpened;
    cv::filter2D(GaussBinaryImage,Sharpened,-1,sharpening_kenel);

    cv::threshold(Sharpened,BinaryImage,ArrowDetectorThresholdThresh,ArrowDetectorThresholdMaxval,cv::THRESH_BINARY);

    # ifdef Imageshow

    cv::imshow("BinaryImage",BinaryImage);
    cv::waitKey(30);

    # endif

    return BinaryImage;
}

Arrow_detector::Arrow_detector(double k):Node("Arrow_detector"),filter_(FilterCorner(k)){
    // this->client_=this->create_client<Imagerequest>("OriginalVideo");
    this->subscription_=this->create_subscription<sensor_msgs::msg::Image>("/sensor/image",10,std::bind(&Arrow_detector::GetImage,this,_1));
    this->RedeemBoxPosition_publisher_=this->create_publisher<interfaces::msg::RedeemBoxPosition>("/arrow_detect/RedeemBoxPosition",10);
    this->label_image_pub_=this->create_publisher<sensor_msgs::msg::Image>("/arrow_detect/label_image",10);
    RCLCPP_INFO(this->get_logger(),"Arrow_detector client created !");

    //定义参数
    try{

    this->declare_parameter<std::string>("Location","/home/pnx/code/Engineering_robot_RM2025_Pnx");
    YAML::Node config = YAML::LoadFile(this->get_parameter("Location").as_string()+"/src/config.yaml");
    cameraMatrix=config["camera"]["camera_matrix"].as<std::vector<double>>();
    distCoeffs=config["camera"]["dist_coeffs"].as<std::vector<double>>();
    for(int i=0;i<9;i++){
        cameraMatrixEigen(i/3,i%3)=cameraMatrix[i];
    }
    for(int i=0;i<8;i++){
        const std::vector<double> & arrowPoints=config["arrow"]["arrowPoints"][i].as<std::vector<double>>();
        objpoints.push_back(cv::Point3d(arrowPoints[0],arrowPoints[1],arrowPoints[2]));
        objpointsEigen.push_back(Eigen::Vector4d(arrowPoints[0],arrowPoints[1],arrowPoints[2],1));
    }
    for(int i=0;i<4;i++){
        const std::vector<double> & redeemptionBoxCornerPoints=config["redeem_box"]["redeemptionBoxCornerPoints"][i].as<std::vector<double>>();
        ObjRedemptionBoxCornerPoint.push_back(cv::Point3d(redeemptionBoxCornerPoints[0],redeemptionBoxCornerPoints[1],redeemptionBoxCornerPoints[2]));
        ObjRedemptionBoxCornerPointEigen.push_back(Eigen::Vector4d(redeemptionBoxCornerPoints[0],redeemptionBoxCornerPoints[1],redeemptionBoxCornerPoints[2],1));
    }
    for(int i=0;i<2;i++){
        const std::vector<double> & line=config["redeem_box"]["line"][i].as<std::vector<double>>();
        Object2cornersEigen.push_back(Eigen::Vector4d(line[0],line[1],line[2],1));
    }
    // for(int i=0;i<9;i++){
    //     CenterToArrowvec(i/3,i%3)=config["redeem_box"]["CenterToArrow"][i].as<double>();
    // }
    frontfacecenter=Eigen::Matrix<double,4,1>(config["redeem_box"]["center"][0].as<double>(),
        config["redeem_box"]["center"][1].as<double>(),
        config["redeem_box"]["center"][2].as<double>(),
        1.0
    );
    signMat<<1,0,0,0,
        0,1,0,0,
        0,0,1,0;
    
    ArrowDetectorPixelNumMax=config["arrow_detect"]["ArrowDetectorPixelNumMax"].as<int>();
    ArrowDetectorPixelNumMin=config["arrow_detect"]["ArrowDetectorPixelNumMin"].as<int>();
    ArrowDetectorLengthWidthRatioMax=config["arrow_detect"]["ArrowDetectorLengthWidthRatioMax"].as<double>();
    ArrowDetectorLengthWidthRatioMin=config["arrow_detect"]["ArrowDetectorLengthWidthRatioMin"].as<double>();
    ArrowDetectorApproxSizeMax=config["arrow_detect"]["ArrowDetectorApproxSizeMax"].as<double>();
    ArrowDetectorApproxSizeMin=config["arrow_detect"]["ArrowDetectorApproxSizeMin"].as<double>();
    ArrowDetectorCannyThreshold1=config["arrow_detect"]["ArrowDetectorCannyThreshold1"].as<double>();
    ArrowDetectorCannyThreshold2=config["arrow_detect"]["ArrowDetectorCannyThreshold2"].as<double>();
    ArrowDetectorHoughRho=config["arrow_detect"]["ArrowDetectorHoughRho"].as<double>();
    ArrowDetectorHoughTheta=config["arrow_detect"]["ArrowDetectorHoughTheta"].as<double>();
    ArrowDetectorHoughThreshold=config["arrow_detect"]["ArrowDetectorHoughThreshold"].as<double>();
    ArrowDetectParallelThreshold=config["arrow_detect"]["ArrowDetectParallelThreshold"].as<double>();
    ArrowDetectorThresholdThresh=config["arrow_detect"]["ArrowDetectorThresholdThresh"].as<double>();
    ArrowDetectorThresholdMaxval=config["arrow_detect"]["ArrowDetectorThresholdMaxval"].as<double>();
    ArrowDetectorThresholdThreshold=config["arrow_detect"]["ArrowDetectorThresholdThreshold"].as<double>();
    ArrowDetectorIterations=config["arrow_detect"]["ArrowDetectorIterations"].as<double>();
    ArrowDetectorapproxPolyDPEpsilon=config["arrow_detect"]["ArrowDetectorapproxPolyDPEpsilon"].as<double>();

    tf_broadcaster_box_to_camera=std::make_shared<tf2_ros::TransformBroadcaster>(this);

    static_tf_broadcaster_camera_to_arm=std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    static_tf_broadcaster_camera_to_map=std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    geometry_msgs::msg::TransformStamped camera_to_arm;

    camera_to_arm.header.frame_id="sensor/camera";
    camera_to_arm.child_frame_id="object/arm";
    camera_to_arm.transform.translation.x=0;
    camera_to_arm.transform.translation.y=0;
    camera_to_arm.transform.translation.z=0;
    camera_to_arm.transform.rotation.w=config["rotate"]["w"].as<double>();
    camera_to_arm.transform.rotation.x=config["rotate"]["x"].as<double>();
    camera_to_arm.transform.rotation.y=config["rotate"]["y"].as<double>();
    camera_to_arm.transform.rotation.z=config["rotate"]["z"].as<double>();

    static_tf_broadcaster_camera_to_arm->sendTransform(camera_to_arm);

    }
    catch(const std::exception& e){
        RCLCPP_ERROR(this->get_logger(),"Fail to load config file : %s",e.what());
        rclcpp::shutdown();
    }

    geometry_msgs::msg::TransformStamped to_map;

    to_map.child_frame_id="sensor/camera";
    to_map.header.frame_id="map";
    to_map.header.stamp=this->now();
    to_map.transform.rotation.w=1;
    to_map.transform.rotation.x=0;
    to_map.transform.rotation.y=0;
    to_map.transform.rotation.z=0;
    to_map.transform.translation.x=0;
    to_map.transform.translation.y=0;
    to_map.transform.translation.z=0;

    static_tf_broadcaster_camera_to_map->sendTransform(to_map);

}

int main (int argc,char* argv[]){
    rclcpp::init(argc,argv);
    auto node=std::make_shared<Arrow_detector>(0.9);

    rclcpp::spin(node);
    rclcpp::shutdown();
}