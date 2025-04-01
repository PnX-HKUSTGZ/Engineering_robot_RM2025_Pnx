#include "RedeemBox_detector.hpp"

void RedeemBox_detector::DetectArrowInit(){
    YAML::Node configDetectArrowInit;
    try{
    configDetectArrowInit=config["RedeemBox_detector"]["Parameters"]["arrow_detect"];

    for(int i=0;i<8;i++){
        const std::vector<double> & arrowPoints=config["RedeemBox_detector"]["KeyPoints"]["arrow"]["arrowPoints"][i].as<std::vector<double>>();
        objpoints.push_back(cv::Point3d(arrowPoints[0],arrowPoints[1],arrowPoints[2]));
        objpointsEigen.push_back(Eigen::Vector4d(arrowPoints[0],arrowPoints[1],arrowPoints[2],1));
    }
    
    ArrowDetectorPixelNumMax=configDetectArrowInit["arrow_detect"]["ArrowDetectorPixelNumMax"].as<int>();
    ArrowDetectorPixelNumMin=configDetectArrowInit["arrow_detect"]["ArrowDetectorPixelNumMin"].as<int>();
    ArrowDetectorLengthWidthRatioMax=configDetectArrowInit["arrow_detect"]["ArrowDetectorLengthWidthRatioMax"].as<double>();
    ArrowDetectorLengthWidthRatioMin=configDetectArrowInit["arrow_detect"]["ArrowDetectorLengthWidthRatioMin"].as<double>();
    ArrowDetectorApproxSizeMax=configDetectArrowInit["arrow_detect"]["ArrowDetectorApproxSizeMax"].as<double>();
    ArrowDetectorApproxSizeMin=configDetectArrowInit["arrow_detect"]["ArrowDetectorApproxSizeMin"].as<double>();
    ArrowDetectorCannyThreshold1=configDetectArrowInit["arrow_detect"]["ArrowDetectorCannyThreshold1"].as<double>();
    ArrowDetectorCannyThreshold2=configDetectArrowInit["arrow_detect"]["ArrowDetectorCannyThreshold2"].as<double>();
    ArrowDetectorHoughRho=configDetectArrowInit["arrow_detect"]["ArrowDetectorHoughRho"].as<double>();
    ArrowDetectorHoughTheta=configDetectArrowInit["arrow_detect"]["ArrowDetectorHoughTheta"].as<double>();
    ArrowDetectorHoughThreshold=configDetectArrowInit["arrow_detect"]["ArrowDetectorHoughThreshold"].as<double>();
    ArrowDetectParallelThreshold=configDetectArrowInit["arrow_detect"]["ArrowDetectParallelThreshold"].as<double>();
    ArrowDetectorThresholdThresh=configDetectArrowInit["arrow_detect"]["ArrowDetectorThresholdThresh"].as<double>();
    ArrowDetectorThresholdMaxval=configDetectArrowInit["arrow_detect"]["ArrowDetectorThresholdMaxval"].as<double>();
    ArrowDetectorThresholdThreshold=configDetectArrowInit["arrow_detect"]["ArrowDetectorThresholdThreshold"].as<double>();
    ArrowDetectorIterations=configDetectArrowInit["arrow_detect"]["ArrowDetectorIterations"].as<double>();
    ArrowDetectorapproxPolyDPEpsilon=configDetectArrowInit["arrow_detect"]["ArrowDetectorapproxPolyDPEpsilon"].as<double>();
    ArrowDetectorLongShortRateMax=configDetectArrowInit["arrow_detect"]["ArrowDetectorLongShortRateMax"].as<double>();
    ArrowDetectorLongShortRateMin=configDetectArrowInit["arrow_detect"]["ArrowDetectorLongShortRateMin"].as<double>();

    }
    catch(const std::exception& e){
        RCLCPP_ERROR(this->get_logger(),"Fail to load config file : %s",e.what());
        rclcpp::shutdown();
    }
}

std::vector<cv::Point2d> RedeemBox_detector::TargetArrow(const cv::Mat & BinaryImage, cv::Mat & Image){
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
    // subopix(this->GreyImage,
    //     ArrowPeaks,
    //     cv::Size(10,10),
    //     cv::Size(-1,-1),
    //     cv::TermCriteria(
    //         cv::TermCriteria::EPS+cv::TermCriteria::COUNT,
    //         150,
    //         0.001
    //         ),
    //     Mask);
    

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

cv::Mat RedeemBox_detector::PreProgress(const cv::Mat & OriginalImage){

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


int RedeemBox_detector::MainDetectArrow(const cv::Mat & OriginalImage){
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