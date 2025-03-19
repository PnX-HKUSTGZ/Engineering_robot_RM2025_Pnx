#include "DetectArrow.hpp"

bool Arrow_detector::PnPsolver(const std::vector<cv::Point2d > & ImagePoints2D,const std::vector<cv::Point3d > & ObjectPoints3D,const std::vector<double> & cameraMatrix,const std::vector<double> & distCoeffs,
    cv::Mat & rvec, cv::Mat & tvec, bool useExtrinsicGuess, int flags){
    cv::Mat cameraMatrixCV=cv::Mat(3,3,CV_64F,const_cast<double *>(cameraMatrix.data())).clone();
    cv::Mat distCoeffsCV=cv::Mat(1,5,CV_64F,const_cast<double *>(distCoeffs.data())).clone();
    
    //pnp homogeneous transfomration matrix
    Eigen::Matrix<double,4,4> rtvecEigen;

    RCLCPP_INFO(this->get_logger(),"start pnp");
    bool PnPsuccess=cv::solvePnP(ObjectPoints3D,
        ImagePoints2D,
        cameraMatrixCV,
        distCoeffsCV,
        rvec,
        tvec,
        useExtrinsicGuess,
        flags);

    std::vector<cv::Mat> rvecs,tvecs;
    bool PnPsuccessed=cv::solvePnPGeneric(
        ObjectPoints3D,
        ImagePoints2D,
        cameraMatrixCV,
        distCoeffsCV,
        rvecs,
        tvecs,
        1,
        cv::SOLVEPNP_IPPE
    );


    RCLCPP_INFO(this->get_logger(),"pnp solves num : %ld",rvecs.size());
    std::stringstream ss_vecs;
    for(std::size_t i=0;i<rvecs.size();i++){
        ss_vecs<<"\n rvecs :"<< rvecs[i]<<std::endl;
        ss_vecs<<"\n tvecs :"<< tvecs[i]<<std::endl;
    }
    RCLCPP_INFO(this->get_logger(),"pnp solves disp: %s",ss_vecs.str().c_str() );

    //check pnp success

    for(int i=0;i<3;i++){
        if(std::isnan(rvec.at<double>(i))||std::isnan(tvec.at<double>(i))){
            RCLCPP_WARN(this->get_logger(),"PNP get NAN!!!!!");
            return 0;
        }
    }
    if(!PnPsuccess||!PnPsuccessed){
        RCLCPP_WARN(this->get_logger(),"PnP fail");
        return 0;
    }
    else{
        RCLCPP_INFO(this->get_logger(),"PnP success");
    }

    std::stringstream rtvecss;
    rtvecss<<"rvec: "<<rvec.at<double>(0)/CV_PI*180<<" "<<rvec.at<double>(1)/CV_PI*180<<" "<<rvec.at<double>(2)/CV_PI*180<<" "<<std::endl;
    rtvecss<<"tvec: "<<tvec.at<double>(0)<<" "<<tvec.at<double>(1)<<" "<<tvec.at<double>(2)<<" "<<std::endl;
    RCLCPP_INFO(this->get_logger(),"%s",rtvecss.str().c_str());
    RCLCPP_INFO(this->get_logger(),"finish pnp");


    //send message and position

    cv::Mat rmat(rvec.size(),rvec.type());
    cv::Rodrigues(rvec,rmat);

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

    // SendBoxPosition(tvec,rvec);

    # ifdef arrow_draw
    DrawPnPResult(OriginalImage_,rvecs[0],tvecs[0],cv::Scalar(225,0,0),2,cv::Point(20,40));
    DrawPnPResult(OriginalImage_,rvecs[1],tvecs[1],cv::Scalar(100,0,200),2,cv::Point(20,100));
    DrawPnPResult(OriginalImage_,rvec,tvec,cv::Scalar(100,100,200),2,cv::Point(20,160));
    auto lable_msg_ptr=cv_bridge::CvImage(std_msgs::msg::Header(),sensor_msgs::image_encodings::BGR8,OriginalImage_).toImageMsg();
    lable_msg_ptr->header.frame_id="/arrow_detect/label_image";
    lable_msg_ptr->header.stamp=this->get_clock()->now();
    this->label_image_pub_->publish(*lable_msg_ptr);

    cv::imshow("pnp result",OriginalImage_);

    cv::waitKey(10);
    # endif

    RCLCPP_INFO(this->get_logger(),"PnPsolver finish");
    return 1;
}

void Arrow_detector::DrawPnPResult(cv::Mat &Image, const cv::Mat & rvec, const cv::Mat & tvec, cv::Scalar color, int thickness, cv::Point textpos){
    //check_valide
    if(!(((rvec.rows==3&&rvec.cols==1)||(rvec.rows==1&&rvec.cols==3))&&
    ((tvec.rows==3&&tvec.cols==1)||(tvec.rows==1&&tvec.cols==3)))){
        RCLCPP_WARN(this->get_logger(),"DrawPnPResult assert fail");
        return;
    }

    for(int i=0;i<3;i++){
        if(std::isnan(rvec.at<double>(i))||
        std::isnan(tvec.at<double>(i))){
            RCLCPP_WARN(this->get_logger(),"DrawPnPResult : tvec or rvec have nan");
            return;
        }
    }

    cv::Mat rmat;
    Eigen::Matrix<double,4,4> rtvecEigen;

    std::stringstream rvecss;
    std::stringstream tvecss; 

    std::vector<cv::Point> RedemptionBox;
    std::vector<cv::Point> Line;

    cv::Rodrigues(rvec,rmat);
    rvecss<<"rvec: "<<rvec.at<double>(0)/CV_PI*180<<" "<<rvec.at<double>(1)/CV_PI*180<<" "<<rvec.at<double>(2)/CV_PI*180<<std::endl;
    tvecss<<"tvec: "<<tvec;

    for(int i=0;i<3;i++){
        for(int e=0;e<3;e++){
            rtvecEigen(i,e)=rmat.at<double>(i,e);
        }
        rtvecEigen(i,3)=tvec.at<double>(i);
    }
    for(int i=0;i<3;i++) rtvecEigen(3, i) = 0.0;
    rtvecEigen(3, 3) = 1.0;

    for(const auto & i : ObjRedemptionBoxCornerPointEigen){
        Eigen::Matrix<double,3,1> coordination=cameraMatrixEigen*signMat*rtvecEigen*i;
        coordination/=coordination(2);
        RedemptionBox.push_back(cv::Point2i(coordination(0),coordination(1)));
    }
    for(const auto & i : Object2cornersEigen){
        Eigen::Matrix<double,3,1> coordination=cameraMatrixEigen*signMat*rtvecEigen*i;
        coordination/=coordination(2);
        Line.push_back(cv::Point2i(coordination(0),coordination(1)));
    }

    Eigen::Matrix<double,4,1> Center3D=rtvecEigen*frontfacecenter;
    std::vector<Eigen::Matrix<double,4,1>> Vectorz3D={rtvecEigen*Eigen::Matrix<double,4,1>(1,0,0,1),
        rtvecEigen*Eigen::Matrix<double,4,1>(0,1,0,1),
        rtvecEigen*Eigen::Matrix<double,4,1>(0,0,1,1)};
    Center3D/=Center3D(3);
    for(auto& i : Vectorz3D){
        i/=i(3);
    }
    Eigen::Matrix<double,3,1> Center2D=cameraMatrixEigen*signMat*Center3D;
    std::vector<Eigen::Matrix<double,3,1>> CenterVectorz2D;
    for(auto Vectorz3D_:Vectorz3D){
        CenterVectorz2D.push_back(cameraMatrixEigen*signMat*Vectorz3D_);
        CenterVectorz2D.back()/=CenterVectorz2D.back()(2);
    }
    Center2D/=Center2D(2);

    std::vector<cv::Point> reput_arrow;
    std::vector<int> indexArrowPoint={0,2,3,1,5,4};
    for(auto i : indexArrowPoint){
        Eigen::Matrix<double,3,1> new_i=cameraMatrixEigen*signMat*rtvecEigen*objpointsEigen[i];
        new_i(0)/=new_i(2);
        reput_arrow.push_back(cv::Point(new_i(0),new_i(1)));
    }

    cv::line(Image,
        cv::Point(Center2D(0),Center2D(1)),
        cv::Point(CenterVectorz2D(0),CenterVectorz2D(1)),
        color);
    cv::circle(Image,cv::Point(Center2D(0),Center2D(1)),1,color,-1);
    cv::putText(Image,"center",
        cv::Point(Center2D(0),Center2D(1)),
        cv::FONT_HERSHEY_SIMPLEX,
        1.0,
        color);

    cv::drawContours(Image,Counters{Line},-1,color,thickness);

    cv::drawContours(Image,Counters{RedemptionBox},-1,color,thickness);

    cv::putText(Image,rvecss.str().c_str(),textpos,cv::FONT_HERSHEY_SIMPLEX,1.0,color);
    cv::putText(Image,tvecss.str().c_str(),cv::Point(textpos.x,textpos.y+25),cv::FONT_HERSHEY_SIMPLEX,1.0,color);
    return;
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
    cv::Mat originalframe=cv_ptr->image,undistortimage;
    cv::undistort(originalframe,undistortimage,[&](){
        cv::Mat ans(cv::Size(3,3),CV_64F);
        for(int i=0;i<9;i++){
            ans.at<double>(i/3,i%3)=this->cameraMatrix[i];
        }
        return ans;
    }(),distCoeffs);
    // originalframe.copyTo(OriginalImage_);
    // RCLCPP_INFO(this->get_logger(), "Get frame");
    MainDetectArrow(undistortimage);
}

std::vector<cv::Point2d> Arrow_detector::TargetArrow(const cv::Mat & BinaryImage, cv::Mat & Image){
    std::vector<cv::Vec2f> lines;
    Counters counters_;
    Counter isarrow;
    std::vector<cv::Point2d> arrowapproxcurve;
    
    cv::findContours(BinaryImage,counters_,cv::RETR_LIST,cv::CHAIN_APPROX_SIMPLE);


    cv::Point2f center;
    std::vector<cv::Point2f> TrianglePeaks;
    float radius;
    cv::Mat MaskedImage;
    std::vector<Slope> slopes;
    std::vector<std::pair<Slope,Slope>> HorizonLinePair;
    std::vector<bool> UsedLineMakePair(20,false);
    double HorizonThreshold=5,RThreshold=10,LThreshold=0;
    int NowMaxSize=0;

    // RCLCPP_INFO(this->get_logger(),"%ld",counters_.size());
    for(auto &counter_ :counters_){
        cv::RotatedRect rotatedrect_;
        try{
            rotatedrect_=cv::minAreaRect(counter_);
        }
        catch (cv::Exception & e){
            RCLCPP_WARN(this->get_logger(),"fail to find circle by minEnclosingCircle : %s",e.what());
            continue;
        }
        // Counter approxcurve;

        double LengthWidthRatio= (std::min(rotatedrect_.size.width,rotatedrect_.size.height)<=eps ? 
            -1 : std::max(rotatedrect_.size.width,rotatedrect_.size.height)/std::min(rotatedrect_.size.width,rotatedrect_.size.height));
        int pixel_num=cv::contourArea(counter_);
        std::vector<cv::Point2d> approxcurve2d;
        cv::approxPolyDP(counter_,approxcurve2d,ArrowDetectorapproxPolyDPEpsilon,1);


        // for(auto i : approxcurve2d) approxcurve.push_back(cv::Point(i.x,i.y));

        std::vector<double> approxcurve_length;
        double Average_4=0,Average_rest=0,approxcurve_length_rate=-1;

        // RCLCPP_INFO(this->get_logger(),"approxcurve2d : %ld",approxcurve2d.size());
        if(approxcurve2d.size()>=6){
            RCLCPP_INFO(this->get_logger(),"turn into approxcurve2d");
            for(int siz_approxcurve2d=approxcurve2d.size(),i=approxcurve2d.size()-1;i>=0;i--){
                approxcurve_length.push_back(DistancePoints(approxcurve2d[i],approxcurve2d[(i+1)%siz_approxcurve2d]));
                // RCLCPP_INFO(this->get_logger(),"length: %lf point: [%lf , %lf] [%lf , %lf]",DistancePoints(approxcurve2d[i],approxcurve2d[(i+1)%siz_approxcurve2d]),
                // approxcurve2d[i].x,approxcurve2d[i].y,
                // approxcurve2d[(i+1)%siz_approxcurve2d].x,approxcurve2d[(i+1)%siz_approxcurve2d].y);
            }
            sort(approxcurve_length.begin(),approxcurve_length.end(),[](auto a,auto b){
                return a>b;
            });
            // for(auto i : approxcurve_length){
            //     RCLCPP_INFO(this->get_logger(),"approxcurve_length[]:%lf",i);
            // }
            for(int i=0;i<4;i++) Average_4+=approxcurve_length[i]/4;
            for(std::size_t i=4;i<approxcurve_length.size();i++) Average_rest+=approxcurve_length[i]/(approxcurve_length.size()-3);
            approxcurve_length_rate=Average_4/Average_rest;
        }

        bool approxcurve_length_rate_check=(
            ArrowDetectorLongShortRateMin<=approxcurve_length_rate&&
            approxcurve_length_rate<=ArrowDetectorLongShortRateMax
        );
        bool pixel_in=(pixel_num>=ArrowDetectorPixelNumMin&&
            pixel_num<=ArrowDetectorPixelNumMax);
        bool lwratio=(ArrowDetectorLengthWidthRatioMin<=LengthWidthRatio&&
            LengthWidthRatio<=ArrowDetectorLengthWidthRatioMax);
        bool approxsize=(std::size_t(ArrowDetectorApproxSizeMin)<=approxcurve2d.size()&&
            approxcurve2d.size()<=std::size_t(ArrowDetectorApproxSizeMax));


        // if(approxcurve.size()<500) continue;

        #ifdef TargetArrowtest

        cv::Mat copy_;
        Image.copyTo(copy_);

        cv::Point2f rotatedrect_points_ptr[4];
        std::vector<cv::Point2f> rotatedrect_points;
        rotatedrect_.points(rotatedrect_points_ptr);
        for(int i=0;i<4;i++){
            rotatedrect_points.push_back(rotatedrect_points_ptr[i]);
        }

        if(pixel_num<500) continue;

        RCLCPP_INFO(this->get_logger(),"LengthWidthRatio : %lf",LengthWidthRatio);
        RCLCPP_INFO(this->get_logger(),"pixel_num : %d",pixel_num);
        RCLCPP_INFO(this->get_logger(),"approxcurve size : %ld",approxcurve2d.size());
        RCLCPP_INFO(this->get_logger(),"approxcurve_length_rate : %lf",approxcurve_length_rate);
        RCLCPP_INFO(this->get_logger(),"Average_4 : %lf",Average_4);
        RCLCPP_INFO(this->get_logger(),"Average_rest : %lf",Average_rest);

        // std::stringstream center_ss;
        // center_ss<<center<<" "<<radius;
        // RCLCPP_INFO(this->get_logger(),"center_ss: %s",center_ss.str().c_str());

        RCLCPP_INFO(this->get_logger(),((pixel_in&&lwratio&&approxsize&&(NowMaxSize<pixel_num)&&approxcurve_length_rate_check) ? "pass" : "not pass"));

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

        if(!(pixel_in&&
            lwratio&&
            approxsize&&
            (NowMaxSize<pixel_num)&&
            approxcurve_length_rate_check)) continue;

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
        for(int i=approxcurve2d.size()-1,siz=approxcurve2d.size();i>=0;i--){
            slopes.push_back(Slope{i,(i+1)%siz,[](cv::Point p1,cv::Point p2){
                double angle=GetAngleAccordingToHorizon(p1,p2);
                return abs(angle-180)<5 ? 0 : angle;}(approxcurve2d[i],approxcurve2d[(i+1)%siz])});
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
        arrowapproxcurve=std::move(approxcurve2d);

        # ifdef arrow_draw

        Counter arrowapproxcurve2i;

        for(auto & i : arrowapproxcurve){
            arrowapproxcurve2i.push_back(cv::Point(i.x,i.y));
        }
        
        cv::drawContours(Image,Counters{isarrow},-1,cv::Scalar(225,225,225),1);
        cv::drawContours(Image,Counters{arrowapproxcurve2i},-1,cv::Scalar(0,225,225),1);

        # endif

    }

    if(isarrow.empty()){
        RCLCPP_WARN(this->get_logger(),"fail to find arrow!");
        return std::vector<cv::Point2d>();
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
        return std::vector<cv::Point2d>();
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
    std::vector<cv::Point2d> ArrowPeaks;
    std::vector<cv::Point2d> subArrowPeaks;

    ArrowPeaks.push_back(arrowapproxcurve[RightAnglePeaks[0]]);
    ArrowPeaks.push_back(arrowapproxcurve[RightAnglePeaks[1]]);
    ArrowPeaks.push_back(cv::Point(0,0));
    ArrowPeaks.push_back(cv::Point(0,0));
    ArrowPeaks.push_back(cv::Point(0,0));
    ArrowPeaks.push_back(cv::Point(0,0));

    cv::Point2d Centerline=ArrowPeaks[0]-cv::Point2d(center.x,center.y);
    for(int i=0;i<=1;i++){
        std::pair<int,int> PointNumPair1=std::make_pair(HorizonLinePair[i].first.p1,HorizonLinePair[i].first.p2);
        std::pair<int,int> PointNumPair2=std::make_pair(HorizonLinePair[i].second.p1,HorizonLinePair[i].second.p2);
        if(PointNumPair1.first==RightAnglePeaks[0]||PointNumPair1.first==RightAnglePeaks[1]) std::swap(PointNumPair1.first,PointNumPair1.second);
        if(PointNumPair2.first==RightAnglePeaks[0]||PointNumPair2.first==RightAnglePeaks[1]) std::swap(PointNumPair2.first,PointNumPair2.second);
        cv::Point2d TargetLine=arrowapproxcurve[PointNumPair1.first]-cv::Point2d(center.x,center.y);
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

    if(ArrowPeaks[3]==cv::Point2d(0,0)||ArrowPeaks[2]==cv::Point2d(0,0)){
        RCLCPP_ERROR(this->get_logger(),"Fail to find midpoint of other two sides");
        return std::vector<cv::Point2d>();
    }
    else RCLCPP_INFO(this->get_logger(),"find midpoint of other two sides successfully");

    # ifdef arrow_draw
    int PeaksCnt=0;
    for(auto i : ArrowPeaks){
        cv::circle(Image,i,1,cv::Scalar(153,156,30),-1);
        std::stringstream ss;ss<<PeaksCnt<<":"<<(i-cv::Point2d(center)).cross(Centerline);PeaksCnt++;
        cv::putText(Image,ss.str(),i,cv::FONT_HERSHEY_SIMPLEX,1.0,cv::Scalar(225,225,225));
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
    subopix(this->GreyImage,
        ArrowPeaks,
        cv::Size(10,10),
        cv::Size(-1,-1),
        cv::TermCriteria(
            cv::TermCriteria::EPS+cv::TermCriteria::COUNT,
            150,
            0.001
            ),
        Mask);
    

    FindPolygonCounterPointsSets(CannyImage,
        LinesPoints,
        ArrowPeaks,
        ArrowDetectorThresholdThreshold,
        Endpoints);

    for(auto &p : Endpoints){
        std::pair<int,int> index=std::make_pair(-1,-1);
        for(int i=0;i<6;i++){
            if(IsPointSame(p.first,ArrowPeaks[i])) index.first=i;
            if(IsPointSame(p.second,ArrowPeaks[i])) index.second=i;
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
        return std::vector<cv::Point2d>();
    }
    else RCLCPP_INFO(this->get_logger(),"size of Endpoints : %ld",Endpoints.size());

    if(EndpointsIndex.size()!=6){
        RCLCPP_WARN(this->get_logger(),"size of Endpoints : %ld which is not equal to 6",EndpointsIndex.size());
        return std::vector<cv::Point2d>();
    }
    else RCLCPP_INFO(this->get_logger(),"size of Endpoints : %ld",EndpointsIndex.size());


    for(std::size_t i=0;i<LinesPoints.size();i++){
        cv::Vec4d line;
        cv::fitLine(LinesPoints[i],line,cv::DIST_L2,0,0.01,0.01);
        // if(EndpointsIndex[i]==std::make_pair(4,5)||
        // EndpointsIndex[i]==std::make_pair(2,3)){
        //     continue;
        // }
        FittedLines.push_back(line);
        // RCLCPP_INFO(this->get_logger(),"Line Info : %lf %lf %lf %lf",line[0],line[1],line[2],line[3]);
    }

    if(FittedLines.size()!=6){
        RCLCPP_WARN(this->get_logger(),"size of FittedLines : %ld which is not equal to 6",FittedLines.size());
        return std::vector<cv::Point2d>();
    }
    else RCLCPP_INFO(this->get_logger(),"size of FittedLines : %ld",FittedLines.size());
    RCLCPP_INFO(this->get_logger(),"finish find lines");

    # ifdef arrow_draw

    DrawLines(Image,FittedLines,cv::Scalar(225,225,225),1);

    # endif

    //第二次迭代顶点储存
    std::vector<cv::Point2d> ArrowPeaks_;
    
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

    std::vector<std::pair<int,int>> LinesOrder={
        std::make_pair(0,2),
        std::make_pair(0,4),
        std::make_pair(1,3),
        std::make_pair(1,5),
        std::make_pair(2,3),
        std::make_pair(4,5)
    };

    std::vector<LineABC> OrderedFittedLines;

    for(int i=0;i<6;i++){
        for(int e=0;e<6;e++){
            if(EndpointsIndex[e]==LinesOrder[i]){
                OrderedFittedLines.push_back(GetLineABC(FittedLines[e]));
            }
        }
    }

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

    if(index1.size()==0||index2.size()==0) return std::vector<cv::Point2d>();

    std::vector<cv::Point> allindex=index1;

    for (auto i : index2) allindex.push_back(i);

    cv::Vec4d line_index;

    cv::fitLine(allindex,line_index,cv::DIST_L2,0,0.01,0.01);

    #ifdef arrow_draw
    DrawLines(Image,std::vector<LineVP>{line_index},cv::Scalar(225,225,225),1);
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
    #endif //twopath_inoneline

    #ifdef arrow_draw

    for(int i=0;i<8;i++){
        // OriginalImage.at<cv::Vec3f>(int(ArrowPeaks_[i].y),int(ArrowPeaks_[i].x))=cv::Vec3f(0,0,0);
        cv::circle(Image,ArrowPeaks_[i],1,cv::Scalar(32,122,225),-1);
        cv::putText(Image,std::to_string(i),ArrowPeaks_[i],cv::FONT_HERSHEY_COMPLEX,1.0,cv::Scalar(32,122,225));
        RCLCPP_INFO(this->get_logger(),"Point %d: [%f,%f]",i,ArrowPeaks_[i].x,ArrowPeaks_[i].y);
    }
    #endif

    # ifndef arrow_draw

    for(int i=0;i<8;i++){
        RCLCPP_INFO(this->get_logger(),"Point %d: [%f,%f]",i,ArrowPeaks_[i].x,ArrowPeaks_[i].y);
    }
    # endif

    # ifdef Imageshow

    cv::imshow("finish one",Image);
    cv::waitKey(33);

    #endif

    if(ArrowPeaks_.size()!=8){
        RCLCPP_ERROR(this->get_logger(),"Fail to find all peaks, size of ArrowPeaks_ : %ld",ArrowPeaks_.size());
        return std::vector<cv::Point2d>();
    }
    else RCLCPP_INFO(this->get_logger(),"find all peaks successfully");

    RCLCPP_INFO(this->get_logger(),"TargetArrow succesfully");

    // LocalCornerOpitimize(BinaryImage,ArrowPeaks_[0],8,2,3,0.04);


    return ArrowPeaks_;
}

bool Arrow_detector::MainDetectArrow(const cv::Mat & OriginalImage){
    #ifdef noMainDetectArrow
    return 0;
    #endif //noMainDetectArrow
    OriginalImage.copyTo(OriginalImage_);
    cv::Mat Binary=PreProgress(OriginalImage);

    std::vector<cv::Point2d> TargetArrowResult=TargetArrow(Binary,OriginalImage_);
    if(!TargetArrowResult.size()){
        RCLCPP_INFO(this->get_logger(),"fail to target arrow.");
        return 0;
    }

    cv::Mat rvec,tvec;
    bool PnPsolverCheck=PnPsolver(TargetArrowResult,objpoints,cameraMatrix,distCoeffs,rvec,tvec,0,cv::SOLVEPNP_IPPE);

    if(PnPsolverCheck) return 1;

    #ifdef SyncPubBoxPos
    pnpressMtx.lock();
    RCLCPP_INFO(this->get_logger(),"QWQWQWQ");
    pnpress.push(PnPresult(tvec,rvec,this->now()));
    RCLCPP_INFO(this->get_logger(),"QWQWQWQ");
    pnpressMtx.unlock();
    #endif

    return 0;
}

cv::Mat Arrow_detector::PreProgress(const cv::Mat & OriginalImage){

    std::vector<cv::Mat> SplitImage;
    //通道顺序为BGR
    cv::split(OriginalImage,SplitImage);

    #ifdef arrow_draw

    cv::imshow("OriginalImage",OriginalImage);

    #endif
    
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

    cv::threshold(GaussBinaryImage,BinaryImage,ArrowDetectorThresholdThresh,ArrowDetectorThresholdMaxval,cv::THRESH_BINARY);

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
    config = YAML::LoadFile(this->get_parameter("Location").as_string()+"/src/config.yaml");
    cameraMatrix=config["camera"]["camera_matrix"].as<std::vector<double>>();
    distCoeffs=config["camera"]["dist_coeffs"].as<std::vector<double>>();
    for(int i=0;i<9;i++){
        cameraMatrixEigen(i/3,i%3)=cameraMatrix[i];
    }
    InverseCameraMatrixEigen=cameraMatrixEigen.inverse();
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
    ArrowDetectorLongShortRateMax=config["arrow_detect"]["ArrowDetectorLongShortRateMax"].as<double>();
    ArrowDetectorLongShortRateMin=config["arrow_detect"]["ArrowDetectorLongShortRateMin"].as<double>();

    tf_broadcaster_box_to_camera=std::make_shared<tf2_ros::TransformBroadcaster>(this);

    static_tf_broadcaster_camera_to_arm=std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);


    geometry_msgs::msg::TransformStamped camera_to_arm;

    camera_to_arm.header.frame_id="map";
    camera_to_arm.child_frame_id="object/arm";
    camera_to_arm.transform.translation.x=config["object_pos"]["arm"]["translation"]["x"].as<double>();
    camera_to_arm.transform.translation.y=config["object_pos"]["arm"]["translation"]["y"].as<double>();
    camera_to_arm.transform.translation.z=config["object_pos"]["arm"]["translation"]["z"].as<double>();
    camera_to_arm.transform.rotation.w=config["object_pos"]["arm"]["rotate"]["w"].as<double>();
    camera_to_arm.transform.rotation.x=config["object_pos"]["arm"]["rotate"]["x"].as<double>();
    camera_to_arm.transform.rotation.y=config["object_pos"]["arm"]["rotate"]["y"].as<double>();
    camera_to_arm.transform.rotation.z=config["object_pos"]["arm"]["rotate"]["z"].as<double>();

    static_tf_broadcaster_camera_to_arm->sendTransform(camera_to_arm);

    }
    catch(const std::exception& e){
        RCLCPP_ERROR(this->get_logger(),"Fail to load config file : %s",e.what());
        rclcpp::shutdown();
    }

    PointCloudeInit();
    #ifdef SyncPubBoxPos
    SyncPubBoxPosInit();
    #endif
}

void Arrow_detector::SyncPubBoxPosInit(){
    YAML::Node syncconfig=config["arrow_detect"]["SyncPubBoxPos"];
    queuesiz=syncconfig["queuesiz"].as<int>();
    syncThresehold=rclcpp::Duration(syncconfig["syncThresehold"].as<std::vector<int>>()[0],
        syncconfig["syncThresehold"].as<std::vector<int>>()[1]);

    respubtimer_=this->create_wall_timer(std::chrono::milliseconds(syncconfig["PubInterval"].as<int>()),std::bind(&Arrow_detector::SyncPubBoxPos,this));
}

void Arrow_detector::SyncPubBoxPos(){
//     std::lock_guard<std::mutex> pnp_guard(pnpressMtx);
//     std::lock_guard<std::mutex> cloud_guard(cloudressMtx);

//     rclcpp::Time now=this->now();
//     while(!pnpress.empty()){
//         if((now-pnpress.front().stamp)>syncThresehold){
//             pnpress.pop();
//         }
//         else{
//             break;
//         }
//     }
//     while(!cloudress.empty()){
//         if((now-cloudress.front().stamp)>syncThresehold){
//             cloudress.pop();
//         }
//         else{
//             break;
//         }
//     }

//     if(pnpress.empty()&&cloudress.empty()){
//         RCLCPP_INFO(this->get_logger(),"SyncPubBoxPos : both are empty return");
//         return;
//     }

//     if(pnpress.empty()){
//         SendBoxPosition(cloudress.front().tvec,cloudress.front().rvec);
//         cloudress.pop();
//         return;
//     }
//     if(pnpress.empty()){
//         SendBoxPosition(cloudress.front().tvec,cloudress.front().rvec);
//         cloudress.pop();
//         return;
//     }

//     //现在的策略：直接发布点云的

//     SendBoxPosition(cloudress.front().tvec,cloudress.front().rvec);
//     cloudress.pop();
//     RCLCPP_INFO(this->get_logger(),"SyncPubBoxPos : send!");
//     return;
}


void Arrow_detector::SendBoxPosition(cv::Mat & tvec,cv::Mat & rvecmat,cv::Mat & OriginalImage){

    CV_Assert((rvecmat.size()==cv::Size(3,3) || rvecmat.size()==cv::Size(3,1) || rvecmat.size()==cv::Size(1,3))&&
        (rvecmat.type()==CV_64F || rvecmat.type()==CV_32F));
    CV_Assert((tvec.size()==cv::Size(3,1)||tvec.size()==cv::Size(1,3))&&
        (tvec.type()==CV_64F ||tvec.type()==CV_32F));
    cv::Mat rmat;
    if(rvecmat.size()==cv::Size(3,1)||rvecmat.size()==cv::Size(1,3)){
        cv::Rodrigues(rvecmat,rmat);
    }
    else{
        rmat=rvecmat;
    }

    #ifdef drawFinalres

        cv::Mat rvec;
        cv::Rodrigues(rmat,rvec);
        DrawPnPResult(OriginalImage,rvec,tvec,cv::Scalar(223,34,100),1,cv::Point(20,40));
        std::stringstream drawFinalressss;
        drawFinalressss<<rvec<<"\n"<<tvec;
        RCLCPP_INFO(this->get_logger(),"drawFinalres : %s",drawFinalressss.str().c_str());
        cv::imshow("Finalres",OriginalImage);
        cv::waitKey(11);

    #endif

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
}

int main (int argc,char* argv[]){
    rclcpp::init(argc,argv);
    auto node=std::make_shared<Arrow_detector>(0.9);

    rclcpp::spin(node);
    rclcpp::shutdown();
}