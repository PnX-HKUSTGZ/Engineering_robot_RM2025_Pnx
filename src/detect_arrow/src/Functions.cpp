#include "DetectArrow.hpp"

double DistanceBetweenPointAndLine(const cv::Point & p,const Line & l){
    double r=l.val[0],theta=l.val[1];
    double t1=(p.y*std::cos(theta)-p.x*std::sin(theta));
    double t0=(r-p.x*std::cos(theta)-p.y*std::sin(theta));
    cv::Point2f p1(r*std::cos(theta)-t1*sin(theta),r*std::sin(theta)+t1*cos(theta));
    cv::Point2f p2(p.x+t0*cos(theta),p.y+t0*sin(theta));
    if(p1!=p2){
        RCLCPP_ERROR(rclcpp::get_logger("DistanceBetweenPointAndLine"),"p1 : [%lf,%lf],p2 : [%lf,%lf],p : [%d,%d]",p1.x,p1.y,p2.x,p2.y,p.x,p.y);
        rclcpp::shutdown();
    }
    RCLCPP_INFO(rclcpp::get_logger("DistanceBetweenPointAndLine"),"r : [%lf],theta : [%lf] ,p1 : [%lf,%lf],p2 : [%lf,%lf],p : [%d,%d], distance: [%lf]",r,theta,p1.x,p1.y,p2.x,p2.y,p.x,p.y,cv::norm(cv::Point2f(p.x,p.y)-p1));
    return cv::norm(cv::Point2f(p.x,p.y)-p1);
}

double DistanceBetweenPointAndLine(const cv::Point2f & p,const Line & l){
    double r=l.val[0],theta=l.val[1];
    double t1=(p.y*std::cos(theta)-p.x*std::sin(theta));
    double t0=(r-p.x*std::cos(theta)-p.y*std::sin(theta));
    cv::Point2f p1(r*std::cos(theta)-t1*sin(theta),r*std::sin(theta)+t1*cos(theta));
    cv::Point2f p2(p.x+t0*cos(theta),p.y+t0*sin(theta));
    if(p1!=p2){
        RCLCPP_ERROR(rclcpp::get_logger("DistanceBetweenPointAndLine"),"p1 : [%lf,%lf],p2 : [%lf,%lf],p : [%lf,%lf]",p1.x,p1.y,p2.x,p2.y,p.x,p.y);
        rclcpp::shutdown();
    }
    RCLCPP_INFO(rclcpp::get_logger("DistanceBetweenPointAndLine"),"r : [%lf],theta : [%lf] ,p1 : [%lf,%lf],p2 : [%lf,%lf],p : [%lf,%lf], distance: [%lf]",r,theta,p1.x,p1.y,p2.x,p2.y,p.x,p.y,cv::norm(p-p1));
    return cv::norm(p-p1);
}


void DrawLines(cv::Mat & img,const Lines & lines, const cv::Scalar& color,
    int thickness, int lineType, int shift){
    for(const auto & i : lines){
        float rho=i[0],theta=i[1];
        double a=std::cos(theta),b=std::sin(theta);
        double x0=a*rho,y0=b*rho;
        cv::Point pt1(cvRound(x0+9000*(-b)),cvRound(y0+9000*(a)));
        cv::Point pt2(cvRound(x0-9000*(-b)),cvRound(y0-9000*(a)));
        cv::line(img,pt1,pt2,color,thickness,lineType,shift);
    }
}

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

bool InPeaksThreshold(const cv::Point & p,const std::vector<cv::Point2f> & Peaks,const double PeaksThreshold){
    for(const auto & i : Peaks){
        if(DistancePoints(p,i)<=PeaksThreshold) return 1;
    }
    return 0;
}

pii PointToPii(const cv::Point & p){
    return std::make_pair(p.x,p.y);
}

void FindContinuePart(const cv::Mat & BinaryImage,std::vector<cv::Point> & Pointset,const cv::Point & StartPoint,const std::vector<cv::Point2f> & Peaks,std::map<std::pair<int,int>,bool> &vis,const double PeaksThreshold){
    static int dx[8]={0,0,1,-1,1,-1,1,-1};
    static int dy[8]={1,-1,0,0,1,1,-1,-1};

    if(BinaryImage.empty()){
        RCLCPP_ERROR(rclcpp::get_logger("FindContinuePart"),"BinaryImage is empty");
        rclcpp::shutdown();
    }
    if(BinaryImage.channels()!=1){
        RCLCPP_ERROR(rclcpp::get_logger("FindContinuePart"),"BinaryImage is not a binary image");
        rclcpp::shutdown();
    }

    int maxy=BinaryImage.rows,maxx=BinaryImage.cols;
    Pointset.push_back(StartPoint);
    vis[PointToPii(StartPoint)]=1;

    std::queue<cv::Point> NowPoints;
    NowPoints.push(StartPoint);

    while(!NowPoints.empty()){
        cv::Point NowPoint=NowPoints.front();
        NowPoints.pop();
        for(int i=0;i<8;i++){
            for(int e=0;e<8;e++){
                int nx=NowPoint.x+dx[i],ny=NowPoint.y+dy[i];
                if(nx<0||nx>=maxx||ny<0||ny>=maxy) continue;
                if(BinaryImage.at<uchar>(ny,nx)==0) continue;
                cv::Point NextPoint(nx,ny);
                if(vis[PointToPii(NextPoint)]) continue;
                if(InPeaksThreshold(NowPoint,Peaks,PeaksThreshold)) continue;
                cv::circle(Arrow_detector::OOriginalImage,NextPoint,1,cv::Scalar(77,55,100),-1);
                Pointset.push_back(NextPoint);
                vis[PointToPii(NextPoint)]=1;
            }
        }
    }

}

void FindPolygonCounterPointsSets(const cv::Mat & BinaryImage,std::vector<std::vector<cv::Point>> & Pointssets,const std::vector<cv::Point2f> & Peaks,const double PeaksThreshold){

    std::map<std::pair<int,int>,bool> vis;

    cv::Rect AOI=cv::boundingRect(BinaryImage);
    for(int i=AOI.x;i<AOI.x+AOI.width;i++){
        for(int e=AOI.y;e<AOI.y+AOI.height;e++){
            cv::Point NowPoint(i,e);
            if(vis[PointToPii(NowPoint)]) continue;
            if(BinaryImage.at<uchar>(e,i)==0) continue;
            if(InPeaksThreshold(NowPoint,Peaks,PeaksThreshold)) continue;
            Pointssets.push_back(std::vector<cv::Point>());
            std::vector<cv::Point> & Pointset=Pointssets.back();
            cv::circle(Arrow_detector::OOriginalImage,NowPoint,1,cv::Scalar(22,33,225),-1);
            FindContinuePart(BinaryImage,Pointset,NowPoint,Peaks,vis,PeaksThreshold);
            cv::imshow("ArrowPeaks",Arrow_detector::OOriginalImage);
            cv::waitKey(0);
            Pointssets.push_back(Pointset);
        }
    }
}

LineABC GetLineABC(const Line & l){
    double r=l.val[0],theta=l.val[1];
    double a=std::cos(theta),b=std::sin(theta);
    double x0=a*r,y0=b*r;
    cv::Point pt1((x0+9000*(-b)),(y0+9000*(a)));
    cv::Point pt2((x0-9000*(-b)),(y0-9000*(a)));
    double A=pt2.y-pt1.y,B=pt1.x-pt2.x,C=pt2.x*pt1.y-pt1.x*pt2.y;
    return LineABC{A,B,C};
}

LineABC GetLineABC(const Line4P & l){
    double x1=l[0],y1=l[1],x2=l[2],y2=l[3];
    double A=y2-y1,B=x1-x2,C=x2*y1-x1*y2;
    return LineABC{A,B,C};
}

LineAL GetLineAL(const Line4P & l){
    return GetLineAL(GetLineABC(l));
}

LineAL GetLineAL(const LineABC & l){
    double A=l.a,B=l.b,C=l.c;
    if(abs(B)<eps) return LineAL{0,float(-C/A)};
    float dis=C/std::sqrt(A*A+B*B);
    float angle=float(CV_PI)/2-std::atan(float(-A/B));
    return LineAL{dis,angle};
}


LineABC ClassicalLeastSquares(const std::vector<cv::Point> & Pointset){
    double sumx=0,sumy=0,sumxy=0,sumx2=0;
    for(const auto & i : Pointset){
        sumx+=i.x;
        sumy+=i.y;
        sumxy+=i.x*i.y;
        sumx2+=i.x*i.x;
    }
    double x_=sumx/Pointset.size(),y_=sumy/Pointset.size();
    double A=(sumxy-sumx*sumy/Pointset.size())/(sumx2-sumx*sumx/Pointset.size());
    double B=y_-A*x_;
    return LineABC{A,-1,B};
}