#include "target_redeem_box/RedeemBox_detector.hpp"

namespace Engineering_robot_RM2025_Pnx{

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

void DrawLines(cv::Mat & img,const std::vector<LineVP> & lines, const cv::Scalar& color,
    int thickness, int lineType, int shift){
    for(const auto & i : lines){
        cv::line(img,cv::Point2d(i[2],i[3])+10000*cv::Point2d(i[0],i[1]),cv::Point2d(i[2],i[3])-10000*cv::Point2d(i[0],i[1]),color,thickness,lineType,shift);
    }
}

//return the angle between line(p1,p2) and horizon
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

template<typename T,typename G>
double DistancePoints(const cv::Point_<T> & p1,const cv::Point_<G> & p2){
    return std::sqrt((p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y));
}

template double DistancePoints<int,int> (const cv::Point_<int> & p1,const cv::Point_<int> & p2);
template double DistancePoints<float,float> (const cv::Point_<float> & p1,const cv::Point_<float> & p2);
template double DistancePoints<double,double> (const cv::Point_<double> & p1,const cv::Point_<double> & p2);
template double DistancePoints<int,double> (const cv::Point_<int> & p1,const cv::Point_<double> & p2);
template double DistancePoints<double,int> (const cv::Point_<double> & p1,const cv::Point_<int> & p2);
template double DistancePoints<float,int> (const cv::Point_<float> & p1,const cv::Point_<int> & p2);
template double DistancePoints<int,float> (const cv::Point_<int> & p1,const cv::Point_<float> & p2);
template double DistancePoints<float,double> (const cv::Point_<float> & p1,const cv::Point_<double> & p2);
template double DistancePoints<double,float> (const cv::Point_<double> & p1,const cv::Point_<float> & p2);

template<typename T>
std::pair<bool,cv::Point_<T> > InPeaksThreshold(const cv::Point & p,const std::vector<cv::Point_<T> > & Peaks,const double PeaksThreshold){
    for(const auto & i : Peaks){
        if(DistancePoints(p,i)<=PeaksThreshold) return std::make_pair(1,i);
    }
    return std::make_pair(0,cv::Point_<T>(-1,-1));
}

template<typename T>
std::pair<T,T> PointToPii(const cv::Point_<T> & p){
    return std::make_pair(p.x,p.y);
}

template<typename T>
bool FindContinuePart(const cv::Mat & BinaryImage,std::vector<cv::Point> & Pointset,const cv::Point & StartPoint,const std::vector<cv::Point_<T> > & Peaks,std::map<std::pair<int,int>,bool> &vis,const double PeaksThreshold,std::pair<cv::Point_<T>,cv::Point_<T> > & endpoints){
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

    // bool meetPeaks=0;

    std::queue<cv::Point> NowPoints;
    NowPoints.push(StartPoint);
    endpoints=std::make_pair(cv::Point_<T>(-1,-1),cv::Point_<T>(-1,-1));
    // RCLCPP_INFO(rclcpp::get_logger("FindContinuePart"),"StartPoint : [%d,%d]",StartPoint.x,StartPoint.y);
    while(!NowPoints.empty()){
        cv::Point NowPoint=NowPoints.front();
        NowPoints.pop();
        for(int i=0;i<8;i++){
            int nx=NowPoint.x+dx[i],ny=NowPoint.y+dy[i];
            cv::Point NextPoint(nx,ny);

            // RCLCPP_INFO(rclcpp::get_logger("FindContinuePart"),"Start NextPoint : [%d,%d]",NextPoint.x,NextPoint.y);

            if(nx<0||nx>=maxx||ny<0||ny>=maxy) continue;
            
            // RCLCPP_INFO(rclcpp::get_logger("FindContinuePart"),"nx<0||nx>=maxx||ny<0||ny>=maxy pass");

            if(BinaryImage.at<uchar>(ny,nx)==0) continue;

            // RCLCPP_INFO(rclcpp::get_logger("FindContinuePart"),"BinaryImage.at<uchar>(ny,nx)==0 pass");

            if(vis[PointToPii(NextPoint)]) continue;

            // RCLCPP_INFO(rclcpp::get_logger("FindContinuePart"),"vis[PointToPii(NextPoint)] pass");
            std::pair<bool,cv::Point_<T> > result=InPeaksThreshold(NowPoint,Peaks,PeaksThreshold);
            if(result.first){
                // meetPeaks=1;
                if(endpoints.first==result.second||endpoints.second==result.second){
                    continue;
                }
                if(abs(endpoints.first.x+1)<=eps){
                    endpoints.first=result.second;
                }
                else if(abs(endpoints.second.x+1)<=eps){
                    endpoints.second=result.second;
                }
                continue;
            }
            // RCLCPP_INFO(rclcpp::get_logger("FindContinuePart"),"InPeaksThreshold pass");

            // RCLCPP_INFO(rclcpp::get_logger("FindContinuePart"),"NowPoint : [%d,%d] %d",NextPoint.x,NextPoint.y,(int)BinaryImage.at<uchar>(ny,nx));
            // Arrow_detector::OOriginalImage.at<cv::Vec3b>(ny,nx)=cv::Vec3b(77,55,100);
            Pointset.push_back(NextPoint);
            NowPoints.push(NextPoint);
            vis[PointToPii(NextPoint)]=1;
        }
    }

    // std::stringstream ss;
    // ss<<endpoints.first<<","<<endpoints.second;
    // RCLCPP_INFO(rclcpp::get_logger("FindContinuePart"),"EndPoint: %s",ss.str().c_str());

    if(endpoints.second.x==-1){
        Pointset.clear();
        return 0;
    }
    return 1;

}
template bool FindContinuePart<int>(const cv::Mat & BinaryImage,std::vector<cv::Point> & Pointset,const cv::Point & StartPoint,const std::vector<cv::Point_<int> > & Peaks,std::map<std::pair<int,int>,bool> &vis,const double PeaksThreshold,std::pair<cv::Point_<int>,cv::Point_<int> > & endpoints);
template bool FindContinuePart<double>(const cv::Mat & BinaryImage,
    std::vector<cv::Point> & Pointset,
    const cv::Point & StartPoint,
    const std::vector<cv::Point_<double> > & Peaks,
    std::map<std::pair<int,int>,bool> &vis,
    const double PeaksThreshold,
    std::pair<cv::Point_<double>,cv::Point_<double> > & endpoints);
template bool FindContinuePart<float>(const cv::Mat & BinaryImage,std::vector<cv::Point> & Pointset,const cv::Point & StartPoint,const std::vector<cv::Point_<float> > & Peaks,std::map<std::pair<int,int>,bool> &vis,const double PeaksThreshold,std::pair<cv::Point_<float>,cv::Point_<float> > & endpoints);


template<typename T> 
void FindPolygonCounterPointsSets(const cv::Mat & BinaryImage,
    std::vector<std::vector<cv::Point>> & Pointssets,
    const std::vector<cv::Point_<T>> & Peaks,
    const double PeaksThreshold,
    std::vector<std::pair<cv::Point_<T>,cv::Point_<T>> >& Endpoints){

    std::map<std::pair<int,int>,bool> vis;

    cv::Rect AOI=cv::boundingRect(BinaryImage);
    for(int i=AOI.x;i<AOI.x+AOI.width;i++){
        for(int e=AOI.y;e<AOI.y+AOI.height;e++){
            cv::Point NowPoint(i,e);
            if(vis[PointToPii(NowPoint)]) continue;
            if(BinaryImage.at<uchar>(e,i)==0) continue;
            if(InPeaksThreshold(NowPoint,Peaks,PeaksThreshold).first) continue;

            std::vector<cv::Point> Pointset;
            std::pair<cv::Point_<T>,cv::Point_<T> > endpoints;

            if (!FindContinuePart(BinaryImage,Pointset,NowPoint,Peaks,vis,PeaksThreshold,endpoints)){
                continue;
            }
            // Arrow_detector::OOriginalImage.at<cv::Vec3b>(e,i)=cv::Vec3b(22,33,225);
            // cv::imshow("QWQWQWQW",Arrow_detector::OOriginalImage);
            // cv::waitKey(0);
            Pointssets.push_back(std::move(Pointset));
            Endpoints.push_back(std::move(endpoints));
        }
    }
}
template void FindPolygonCounterPointsSets<double>(const cv::Mat & BinaryImage,
    std::vector<std::vector<cv::Point>> & Pointssets,
    const std::vector<cv::Point_<double> > & Peaks,
    const double PeaksThreshold,
    std::vector<std::pair<cv::Point_<double>,cv::Point_<double>> > & Endpoints);
template void FindPolygonCounterPointsSets<int>(const cv::Mat & BinaryImage,
    std::vector<std::vector<cv::Point>> & Pointssets,
    const std::vector<cv::Point_<int> > & Peaks,
    const double PeaksThreshold,
    std::vector<std::pair<cv::Point_<int>,cv::Point_<int>> > & Endpoints);

LineABC GetLineABC(const Line & l){
    double r=l.val[0],theta=l.val[1];
    double a=std::cos(theta),b=std::sin(theta);
    double x0=a*r,y0=b*r;
    cv::Point pt1((x0+9000*(-b)),(y0+9000*(a)));
    cv::Point pt2((x0-9000*(-b)),(y0-9000*(a)));
    double A=pt2.y-pt1.y,B=pt1.x-pt2.x,C=pt2.x*pt1.y-pt1.x*pt2.y;
    return LineABC{A,B,C};
}

LineABC GetLineABC(const LineVP & l){
    double x1=l[2],y1=l[3],x2=l[2]+l[0]*5,y2=l[3]+l[1]*5;
    double A=y2-y1,B=x1-x2,C=x2*y1-x1*y2;
    return LineABC{A,B,C};
}

LineAL GetLineAL(const LineVP & l){
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

template<typename T>
void GetLinesIntersections(const std::vector<LineVP> & lines,std::vector<cv::Point_<T> > & Intersections){
    int siz=lines.size();
    for(int i=0;i<siz;i++){
        for(int e=i+1;e<siz;e++){
            if(abs(lines[i][0]-lines[e][0])<eps&&abs(lines[i][1]-lines[e][1])<eps){
                continue;
                //平行
            }
            double a1=lines[i][0],b1=lines[i][1],x1=lines[i][2],y1=lines[i][3];
            double a2=lines[e][0],b2=lines[e][1],x2=lines[e][2],y2=lines[e][3];

            double t1=(b1*(x2-x1)-a1*(y2-y1))/(b2*a1-a2*b1);
            double t2=(b2*(x1-x2)-a2*(y1-y2))/(b1*a2-a1*b2);
            double x=x2+t1*a2,y=y2+t1*b2;
            double x_=x1+t2*a1,y_=y1+t2*b1;

            if(abs(x-x_)>eps||abs(y-y_)>eps){
                RCLCPP_ERROR(rclcpp::get_logger("GetLinesIntersections"),"x : [%lf],y : [%lf],x_ : [%lf],y_ : [%lf]",x,y,x_,y_);
                // rclcpp::shutdown();
            }
            Intersections.push_back(cv::Point2f(x,y));
        }
    }
}

template void GetLinesIntersections<int>(const std::vector<LineVP> & lines,std::vector<cv::Point_<int> > & Intersections);
template void GetLinesIntersections<double>(const std::vector<LineVP> & lines,std::vector<cv::Point_<double> > & Intersections);
template void GetLinesIntersections<float>(const std::vector<LineVP> & lines,std::vector<cv::Point_<float> > & Intersections);

cv::Point2f GetLineIntersections(const LineVP & line1,const LineVP & line2){
    double a1=line1[0],b1=line1[1],x1=line1[2],y1=line1[3];
    double a2=line2[0],b2=line2[1],x2=line2[2],y2=line2[3];

    if(abs(a1-a2)<eps&&abs(b1-b2)<eps){
        RCLCPP_ERROR(rclcpp::get_logger("GetLineIntersections"),"two lines parallel! line1 : [%lf,%lf,%lf,%lf], line2 : [%lf,%lf,%lf,%lf]",
        a1,b1,x1,y1,
        a2,b2,x2,y2);
        rclcpp::shutdown();
    }

    double t1=(b1*(x2-x1)-a1*(y2-y1))/(b2*a1-a2*b1);
    double x=x2+t1*a2,y=y2+t1*b2;
    
    return cv::Point2f(x,y);
}

cv::Vec4d rotationMatrixToQuaternion(const cv::Mat& R) {
    // 确保输入是3x3的浮点矩阵
    CV_Assert(R.size() == cv::Size(3, 3) && (R.type() == CV_64F || R.type() == CV_32F));

    double trace = R.at<double>(0,0) + R.at<double>(1,1) + R.at<double>(2,2);
    cv::Vec4d q;

    if (trace > 0) {
        double s = 0.5 / sqrt(trace + 1.0);
        q[0] = 0.25 / s;
        q[1] = (R.at<double>(2,1) - R.at<double>(1,2)) * s;
        q[2] = (R.at<double>(0,2) - R.at<double>(2,0)) * s;
        q[3] = (R.at<double>(1,0) - R.at<double>(0,1)) * s;
    } else {
        if (R.at<double>(0,0) > R.at<double>(1,1) && R.at<double>(0,0) > R.at<double>(2,2)) {
            double s = 2.0 * sqrt(1.0 + R.at<double>(0,0) - R.at<double>(1,1) - R.at<double>(2,2));
            q[0] = (R.at<double>(2,1) - R.at<double>(1,2)) / s;
            q[1] = 0.25 * s;
            q[2] = (R.at<double>(0,1) + R.at<double>(1,0)) / s;
            q[3] = (R.at<double>(0,2) + R.at<double>(2,0)) / s;
        } else if (R.at<double>(1,1) > R.at<double>(2,2)) {
            double s = 2.0 * sqrt(1.0 + R.at<double>(1,1) - R.at<double>(0,0) - R.at<double>(2,2));
            q[0] = (R.at<double>(0,2) - R.at<double>(2,0)) / s;
            q[1] = (R.at<double>(0,1) + R.at<double>(1,0)) / s;
            q[2] = 0.25 * s;
            q[3] = (R.at<double>(1,2) + R.at<double>(2,1)) / s;
        } else {
            double s = 2.0 * sqrt(1.0 + R.at<double>(2,2) - R.at<double>(0,0) - R.at<double>(1,1));
            q[0] = (R.at<double>(1,0) - R.at<double>(0,1)) / s;
            q[1] = (R.at<double>(0,2) + R.at<double>(2,0)) / s;
            q[2] = (R.at<double>(1,2) + R.at<double>(2,1)) / s;
            q[3] = 0.25 * s;
        }
    }

    // 归一化
    double norm = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    q /= norm;
    return q;
}

void subopix(const cv::Mat& GrayImage, std::vector<cv::Point2d>& pointset, cv::Size winSize, cv::Size zeroZone, cv::TermCriteria criteria, cv::InputArray mask){
    RCLCPP_INFO(rclcpp::get_logger("subopix"), "%d", GrayImage.type());
    CV_Assert(GrayImage.type()==CV_8U||GrayImage.type()==CV_32F||GrayImage.type()==CV_32FC1||GrayImage.type()==CV_64F||GrayImage.type()==CV_64FC1);

    return;

    cv::Mat masked;
    cv::copyTo(GrayImage,masked,mask);

    
    RCLCPP_INFO(rclcpp::get_logger("subopix"), "%ld", pointset.size());
    // cv::goodFeaturesToTrack()

    cv::cornerSubPix(masked,pointset,winSize,zeroZone,criteria);

    return;
}

template<typename T,typename G>
bool IsPointSame(cv::Point_<T> point1,cv::Point_<G> point2){
    cv::Point2d diff=cv::Point2d(point1.x-point2.x,point1.y-point2.y);
    return (diff.x*diff.x+diff.y*diff.y) <= 1e-9;
}

template bool IsPointSame<double, double>(cv::Point_<double> point1,cv::Point_<double> point2);
template bool IsPointSame<double, int>(cv::Point_<double> point1,cv::Point_<int> point2);


double CalculatePlantEquality(Eigen::VectorXf plant,std::vector<double> coff,int emptyplace){
    assert(coff.size()==2&&emptyplace<=2&&plant.rows()==4);
    double lin=-plant(3);
    for(int i=0;i<3;i++){
        if(plant(i)==0){
            return NAN;
        }
        if(i==emptyplace) continue;
        lin-=plant(i)*coff[i];
    }
    double ans=lin/plant(emptyplace);

    return ans;
}

bool KabschAlgorithm(const std::vector<cv::Point3d> &Source,
    const std::vector<cv::Point3d>& Target,
    cv::Mat & tvec,
    cv::Mat & rvec){

    RCLCPP_INFO(rclcpp::get_logger("KabschAlgorithm"),"Source.size() : %d",Source.size());
    RCLCPP_INFO(rclcpp::get_logger("KabschAlgorithm"),"Target.size() : %d",Target.size());
    if (Source.size()!=Target.size()){
        return 1;
    }
    tvec=cv::Mat(cv::Size(3,1),CV_64F);
    rvec=cv::Mat(cv::Size(3,1),CV_64F);

    std::vector<Eigen::Matrix<double,3,1>> SourceEigen(Source.size());
    std::vector<Eigen::Matrix<double,3,1>> TargetEigen(Target.size());
    Eigen::Matrix<double,3,1> source_centroid;
    Eigen::Matrix<double,3,1> target_centroid;
    std::vector<Eigen::Matrix<double,3,1>> centered_source(Source.size());
    std::vector<Eigen::Matrix<double,3,1>> centered_target(Source.size());

    for(int siz=Source.size(),i=0;i<siz;i++){
        SourceEigen[i]<<Source[i].x,Source[i].y,Source[i].z;
        TargetEigen[i]<<Target[i].x,Target[i].y,Target[i].z;
        source_centroid+=SourceEigen[i]/siz;
        target_centroid+=TargetEigen[i]/siz;
    }

    for(int siz=Source.size(),i=0;i<siz;i++){
        centered_source[i]=SourceEigen[i]-source_centroid;
        centered_target[i]=TargetEigen[i]-target_centroid;
    }

    Eigen::Matrix<double,3,3> H=Eigen::Matrix<double,3,3>::Zero();

    for(int siz=Source.size(),i=0;i<siz;i++){
        H+=centered_source[i]*centered_target[i].transpose();
    }

    // SVD
    Eigen::JacobiSVD<Eigen::Matrix<double,3,3>> SVD(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = SVD.matrixU();
    Eigen::Matrix3d V = SVD.matrixV();

    // Handle reflection (ensure proper rotation)
    Eigen::Matrix3d S = Eigen::Matrix3d::Identity();
    if (U.determinant() * V.determinant() < 0) {
        S(2, 2) = -1; // Flip the sign of the last column of V
    }

    Eigen::Matrix<double,3,3> rotation;
    Eigen::Matrix<double,3,1> translation;
    cv::Mat rotation33(cv::Size(3,3),CV_64F);

    // Compute rotation matrix
    rotation = V * S * U.transpose();

    // Compute translation vector
    translation = target_centroid - rotation * source_centroid;

    for(int i=0;i<3;i++){
        for(int e=0;e<3;e++){
            rotation33.at<double>(i,e)=rotation(i,e);
        }
        tvec.at<double>(i)=translation(i);
    }
    cv::Rodrigues(rotation33,rvec);

    RCLCPP_INFO(rclcpp::get_logger("KabschAlgorithm"),"finish!");
    return 0;
}

template<typename T,typename G>
cv::Point_<double> Point3to2Transform(const Eigen::Matrix<G,3,3> & cameraMatrixEigen,const cv::Point3_<T> &point){
    Eigen::Matrix<G,4,1> pointEigen;
    Eigen::Matrix<G,3,1> ansEigen;
    Eigen::Matrix<G,3,4> signMat;
    signMat<<1,0,0,0,
    0,1,0,0,
    0,0,0,1;
    pointEigen<<point.x,point.y,point.z,1;
    ansEigen=cameraMatrixEigen*signMat*pointEigen;
    ansEigen/=ansEigen(2);
    return cv::Point2d(ansEigen(0),ansEigen(1));
}

template cv::Point_<double> Point3to2Transform< int, double>(const Eigen::Matrix<double,3,3> & cameraMatrixEigen,const cv::Point3_<int> &point);
template cv::Point_<double> Point3to2Transform< float, float> (const Eigen::Matrix<float,3,3> & cameraMatrixEigen,const cv::Point3_<float> &point);
template cv::Point_<double> Point3to2Transform< double, float>(const Eigen::Matrix<float,3,3> & cameraMatrixEigen,const cv::Point3_<double> &point);
template cv::Point_<double> Point3to2Transform< double, double>(const Eigen::Matrix<double,3,3> & cameraMatrixEigen,const cv::Point3_<double> &point);

template<typename T,typename G>
std::vector<cv::Point_<double>> Points3to2Transform(const Eigen::Matrix<G,3,3> & cameraMatrixEigen,const std::vector<cv::Point3_<T>> &points){
    std::vector<cv::Point_<double>> ans;
    for(auto & i : points){
        ans.push_back(Point3to2Transform(cameraMatrixEigen,i));
    }
    return ans;
}

template std::vector<cv::Point_<double>> Points3to2Transform< int, double>(const Eigen::Matrix<double,3,3> & cameraMatrixEigen,const std::vector<cv::Point3_<int>> &points);
template std::vector<cv::Point_<double>> Points3to2Transform< float, float> (const Eigen::Matrix<float,3,3> & cameraMatrixEigen,const std::vector<cv::Point3_<float>> &points);
template std::vector<cv::Point_<double>> Points3to2Transform< double, float>(const Eigen::Matrix<float,3,3> & cameraMatrixEigen,const std::vector<cv::Point3_<double>> &points);
template std::vector<cv::Point_<double>> Points3to2Transform< double, double>(const Eigen::Matrix<double,3,3> & cameraMatrixEigen,const std::vector<cv::Point3_<double>> &points);


void DrawRotatedRect(cv::Mat &Image,const cv::RotatedRect& rect, const cv::Scalar &color, int thinkness){
    cv::drawContours(Image,Counters{
        [&](){
            Counter ans;
            cv::Point2f rotatedrect_points_ptr[4];
            rect.points(rotatedrect_points_ptr);
            for(int i=0;i<4;i++){
                ans.push_back(cv::Point(rotatedrect_points_ptr[i].x,rotatedrect_points_ptr[i].y));
            }
            return ans;
        }()
    },-1,color,thinkness);
}

bool isPointInsideRotatedRect(const cv::Point2f& pt, const cv::RotatedRect& rect) {
    // 将点平移到以矩形中心为原点的坐标系
    cv::Point2f translated = pt - rect.center;
    
    // 计算旋转角度（弧度）及其三角函数值
    float angle = rect.angle * CV_PI / 180.0f;
    float cosA = std::cos(-angle);  // 逆旋转角度
    float sinA = std::sin(-angle);
    
    // 应用逆旋转变换
    float xRotated = translated.x * cosA - translated.y * sinA;
    float yRotated = translated.x * sinA + translated.y * cosA;
    
    // 检查是否在未旋转的矩形范围内
    float halfWidth = rect.size.width / 2.0f;
    float halfHeight = rect.size.height / 2.0f;
    return (std::abs(xRotated) <= halfWidth && std::abs(yRotated) <= halfHeight);
}

PnPresult::PnPresult(const cv::Mat &tvec_,const cv::Mat &rvec_,rclcpp::Time tim){
    tvec_.copyTo(tvec);
    rvec_.copyTo(rvec);
    stamp=tim;
}


template<typename T>
void CombineCounters(const std::vector<std::vector<cv::Point_<T>>> & counters,std::vector<cv::Point_<T>> & output){
    output.clear();
    for(auto & i : counters){
        for(auto & e : i){
            output.push_back(e);
        }
    }
    return;
}

template void CombineCounters<int>(const std::vector<std::vector<cv::Point_<int>>> & counters,std::vector<cv::Point_<int>> & output);
template void CombineCounters<double>(const std::vector<std::vector<cv::Point_<double>>> & counters,std::vector<cv::Point_<double>> & output);

void DrawTrangle(cv::Mat & Image,Counter2f & CornerPoints,cv::Scalar color,int thickness){
    cv::line(Image,CornerPoints[0],CornerPoints[1],color,thickness);
    cv::line(Image,CornerPoints[1],CornerPoints[2],color,thickness);
    cv::line(Image,CornerPoints[2],CornerPoints[0],color,thickness);
}

CombGenerator::CombGenerator(int n_,int m_){
    assert(n_>0&&m_>0&&n_>=m_);
    n=n_;
    m=m_;
    recorder=std::vector<int>(m_,-1);

}

bool CombGenerator::update(){
    if(recorder[0]==-1){
        for(int i=0;i<m;i++){
            recorder[i]=(i);
        }
        return 0;
    }
    int index=m-1;
    while(index>=0&&recorder[index]==n-m+index) index--;
    if(index==-1) return 1;
    recorder[index]++;
    for(int i=index+1;i<m;i++){
        recorder[i]=recorder[i-1]+1;
    }
    return 0;
}


std::vector<int> CombGenerator::get_next(){
    if(update()){
        return std::vector<int>();
    }
    else{
        // for(auto i : recorder){
        //     RCLCPP_INFO(rclcpp::get_logger("CombGener"),"recorder : %ld",i);
        // }
        return recorder;
    }
}

template<typename T>
void AppendCounters(const std::vector<std::vector<cv::Point_<T>>> & source,std::vector<cv::Point_<T>> & target){
    for(auto & i : source){
        for(auto & e : i ){
            target.push_back(e);
        }
    }
}

template void AppendCounters<int>(const std::vector<std::vector<cv::Point_<int>>> & source,std::vector<cv::Point_<int>> & target);
template void AppendCounters<double>(const std::vector<std::vector<cv::Point_<double>>> & source,std::vector<cv::Point_<double>> & target);

void DrawRect(cv::Mat & Image, const cv::Rect & rect, cv::Scalar color, int thickness){
    cv::line(Image,cv::Point(rect.x,rect.y),cv::Point(rect.x,rect.y+rect.height),color,thickness);
    cv::line(Image,cv::Point(rect.x,rect.y),cv::Point(rect.x+rect.width,rect.y),color,thickness);
    cv::line(Image,cv::Point(rect.x+rect.width,rect.y+rect.height),cv::Point(rect.x,rect.y+rect.height),color,thickness);
    cv::line(Image,cv::Point(rect.x+rect.width,rect.y+rect.height),cv::Point(rect.x+rect.width,rect.y),color,thickness);
}


pcl::PointCloud<pcl::PointXYZ>::Ptr removeHiddenPoints(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_camera_frame,
    int img_width, int img_height,
    float fx, float fy, float cx, float cy,
    float near_plane)
{
    if (!cloud_camera_frame || cloud_camera_frame->empty()) {
        std::cerr << "Input cloud (in camera frame) is empty or null." << std::endl;
        return pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
    }

    // --- 1. Initialize Depth Buffer and Index Buffer ---
    std::vector<std::vector<float>> depth_buffer(
        img_height, std::vector<float>(img_width, std::numeric_limits<float>::max()));

    std::vector<std::vector<int>> index_buffer(
        img_height, std::vector<int>(img_width, -1));

    // --- 2. Project Points and Populate Buffers ---
    for (size_t i = 0; i < cloud_camera_frame->points.size(); ++i) {
        const auto& p = cloud_camera_frame->points[i];

        // Get point coordinates in camera frame (already assumed)
        float X_c = p.x;
        float Y_c = p.y;
        float Z_c = p.z; // Depth

        // Ignore points behind or at the near plane
        if (Z_c <= near_plane) {
            continue;
        }
        // Also ignore points that might project behind the camera (Z_c should be positive for points in front)
        if (Z_c <= 0) {
             continue; // Should already be handled by near_plane if near_plane > 0
        }


        // Perspective Projection to pixel coordinates (u, v)
        // u = fx * (X_c / Z_c) + cx
        // v = fy * (Y_c / Z_c) + cy
        float u_float = fx * (X_c / Z_c) + cx;
        float v_float = fy * (Y_c / Z_c) + cy;

        // Round to nearest integer pixel coordinates
        int u = static_cast<int>(std::round(u_float));
        int v = static_cast<int>(std::round(v_float));

        // Check if the projected point is within the image bounds
        if (u >= 0 && u < img_width && v >= 0 && v < img_height) {
            // If this point is closer than the one currently stored at (v, u)
            if (Z_c < depth_buffer[v][u]) {
                depth_buffer[v][u] = Z_c;
                index_buffer[v][u] = static_cast<int>(i); // Store original point index
            }
        }
    }

    // --- 3. Collect Indices of Visible Points ---
    std::vector<int> visible_indices;
    visible_indices.reserve(img_width * img_height);

    for (int v = 0; v < img_height; ++v) {
        for (int u = 0; u < img_width; ++u) {
            if (index_buffer[v][u] != -1) {
                visible_indices.push_back(index_buffer[v][u]);
            }
        }
    }

    // --- 4. Create Visible Point Cloud ---
    pcl::PointCloud<pcl::PointXYZ>::Ptr visible_cloud_camera_frame(new pcl::PointCloud<pcl::PointXYZ>());
    visible_cloud_camera_frame->points.reserve(visible_indices.size());

    for (int index : visible_indices) {
        visible_cloud_camera_frame->points.push_back(cloud_camera_frame->points[index]);
    }

    visible_cloud_camera_frame->width = visible_cloud_camera_frame->points.size();
    visible_cloud_camera_frame->height = 1;
    visible_cloud_camera_frame->is_dense = cloud_camera_frame->is_dense;

    return visible_cloud_camera_frame;
}

float PlaneDistanceToOrigin(const PlaneData& plane_data) {
    static auto logger=rclcpp::get_logger("PlaneDistanceToOrigin");
    // Ensure the coefficients vector has the correct size for a plane (a, b, c, d)
    if (plane_data.coefficients.values.size() != 4) {
        RCLCPP_ERROR_STREAM(logger, "Error: Invalid number of plane coefficients (" << plane_data.coefficients.values.size() << " instead of 4)." );
        return std::numeric_limits<float>::quiet_NaN(); // Return NaN for invalid input
    }

    float a = plane_data.coefficients.values[0];
    float b = plane_data.coefficients.values[1];
    float c = plane_data.coefficients.values[2];
    float d = plane_data.coefficients.values[3];

    // Calculate the magnitude of the normal vector (a, b, c)
    float normal_magnitude_sq = a*a + b*b + c*c;

    // Check if the normal vector is zero (which would mean it's not a valid plane normal)
    // Use a small epsilon for floating point comparison with zero
    const float epsilon = 1e-6f;
    if (normal_magnitude_sq < epsilon * epsilon) {
        if (std::abs(d) < epsilon) {
             // If d is also close to zero, it might represent the entire space (0=0),
             // which includes the origin. Distance is arguably 0.
             return 0.0f;
         } else {
             // If d is non-zero, 0=d is a contradiction. Not a valid plane.
            RCLCPP_ERROR(logger,"invalid plane!");
            return std::numeric_limits<float>::quiet_NaN(); // Return NaN
         }
    }

    float normal_magnitude = std::sqrt(normal_magnitude_sq);

    // Distance from origin (0, 0, 0) to ax + by + cz + d = 0 is |d| / sqrt(a^2 + b^2 + c^2)
    float distance = std::fabs(d) / normal_magnitude;

    return distance;
}

float IntersectionDistanceAlongZ(const PlaneData& plane_data) {
    static auto logger=rclcpp::get_logger("IntersectionDistanceAlongZ");
    // Ensure the coefficients vector has the correct size for a plane (a, b, c, d)
    if (plane_data.coefficients.values.size() != 4) {
        RCLCPP_ERROR_STREAM(logger, "Error: Invalid number of plane coefficients (" << plane_data.coefficients.values.size() << " instead of 4).");
        return std::numeric_limits<float>::quiet_NaN(); // Return NaN for invalid input
    }

    float a = plane_data.coefficients.values[0];
    float b = plane_data.coefficients.values[1];
    float c = plane_data.coefficients.values[2];
    float d = plane_data.coefficients.values[3];

    const float epsilon = 1e-6f; // Tolerance for floating point comparison with zero

    // Check if the coefficient 'c' is close to zero
    if (std::fabs(c) < epsilon) {
        // Plane is parallel to the Z-axis (or is 0=0 space)
        // If c is zero, the ray (0,0,t) substituted into ax+by+cz+d=0 becomes d=0.
        // If d is also zero, the plane contains the Z-axis ray. No unique intersection point.
        // If d is non-zero, there's no solution for t, no intersection.
        RCLCPP_ERROR(logger, "Warning: Coefficient 'c' is close to zero. Plane is parallel to Z-axis." );
        // In both c=0, d=0 and c=0, d!=0 cases, there is no unique intersection distance along the ray.
        return std::numeric_limits<float>::quiet_NaN();
    }

    // If c is not zero, calculate t = -d / c
    float t = -d / c;

    // Check if the intersection point is on the positive Z-axis (t >= 0)
    if (t >= -epsilon) { // Use epsilon to handle floating point results very close to zero
        return t; // The distance is t
    } else {
        // Intersection is on the negative Z-axis
        RCLCPP_WARN_STREAM(logger, "Info: Intersection point is on the negative Z-axis (t = " << t << "). No intersection along positive ray." );
        return std::numeric_limits<float>::quiet_NaN(); // No intersection along the positive Z-axis ray
    }
}

/**
 * @brief Determines the plane defined by three given points.
 *        Input points are Eigen::Vector3d (Eigen::Matrix<double, 3, 1>).
 *
 * The plane equation is ax + by + cz + d = 0, where (a, b, c) is the normalized normal vector.
 *
 * @param p1 The first point (Eigen::Vector3d).
 * @param p2 The second point (Eigen::Vector3d).
 * @param p3 The third point (Eigen::Vector3d).
 * @param epsilon Tolerance for checking if points are collinear (normal vector is close to zero).
 * @return A std::vector<double> containing the coefficients {a, b, c, d} if successful (size 4).
 *         Returns an empty std::vector<double> if the three points are collinear.
 */
std::vector<double> determinePlaneFromThreePoints(
    const Eigen::Vector3d& p1,
    const Eigen::Vector3d& p2,
    const Eigen::Vector3d& p3,
    double epsilon) // Using double precision epsilon
{
    // Calculate two vectors on the plane using double precision
    Eigen::Vector3d v12 = p2 - p1;
    Eigen::Vector3d v13 = p3 - p1;

    // Calculate the normal vector as the cross product (double precision)
    Eigen::Vector3d normal = v12.cross(v13);

    // Check if the normal vector is close to zero (points are collinear) using double precision norm
    if (normal.norm() < epsilon) {
        RCLCPP_ERROR(rclcpp::get_logger("determinePlaneFromThreePoints"),"Error: The three points are collinear. Cannot determine a unique plane.");
        return std::vector<double>(); // Return empty vector to indicate failure
    }

    // Normalize the normal vector (double precision)
    Eigen::Vector3d normalized_normal = normal.normalized();

    // Calculate the d coefficient: d = -(a*x + b*y + c*z) for any point (x,y,z) on the plane
    // Using p1 and double precision dot product: d = -(normalized_normal . p1)
    double d = -normalized_normal.dot(p1);

    // Store normalized a, b, c coefficients and d into a vector<double>
    std::vector<double> coefficients;
    coefficients.push_back(normalized_normal.x()); // a
    coefficients.push_back(normalized_normal.y()); // b
    coefficients.push_back(normalized_normal.z()); // c
    coefficients.push_back(d);                    // d

    return coefficients; // Return the vector of coefficients
}

/**
 * @brief Calculates the distance from a point to a plane defined by a vector of coefficients.
 *
 * The plane is defined by coefficients (a, b, c, d) in the vector for the equation ax + by + cz + d = 0.
 * It assumes (a, b, c) is a normalized normal vector, so sqrt(a^2 + b^2 + c^2) ≈ 1.
 * The distance from point (x0, y0, z0) is |a*x0 + b*y0 + c*z0 + d|.
 *
 * @param point The 3D point (x0, y0, z0).
 * @param plane_coefficients A vector of doubles containing the plane coefficients {a, b, c, d}.
 *                           Assumed to have size 4 and (a, b, c) normalized.
 * @return The distance from the point to the plane. Returns NaN if coefficients vector is invalid.
 */
float PointToPlaneDistance(
    const pcl::PointXYZ& point,
    const std::vector<double>& plane_coefficients)
{
    // Ensure the coefficients vector has the correct size for a plane (a, b, c, d)
    if (plane_coefficients.size() != 4) {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("PointToPlaneDistance"),"Error: Invalid number of plane coefficients ("<< plane_coefficients.size() << " instead of 4)." );
        return std::numeric_limits<float>::quiet_NaN(); // Return NaN for invalid input
    }

    // Use double for intermediate calculations for better precision, cast to float at the end if needed.
    // However, since pcl::PointXYZ uses float, the precision might be limited by the input point anyway.
    // Let's use float for consistency with pcl::PointXYZ. If higher precision is needed,
    // consider taking Eigen::Vector3d as input instead of pcl::PointXYZ.
    float a = static_cast<float>(plane_coefficients[0]);
    float b = static_cast<float>(plane_coefficients[1]);
    float c = static_cast<float>(plane_coefficients[2]);
    float d = static_cast<float>(plane_coefficients[3]);

    float x0 = point.x;
    float y0 = point.y;
    float z0 = point.z;

    // The signed distance is a*x0 + b*y0 + c*z0 + d
    // Since (a, b, c) are assumed normalized, sqrt(a^2+b^2+c^2) is 1.
    // We take the absolute value for the non-negative distance.
    float signed_distance = a * x0 + b * y0 + c * z0 + d;
    float distance = std::fabs(signed_distance); // Use std::fabs for float

    return distance;
}

float PointToPlaneDistance(
    const pcl::PointXYZ& point,
    const std::vector<float>& plane_coefficients){
        std::vector<double> coe;
        for(auto i : plane_coefficients){
            coe.push_back(i);
        }
        return PointToPlaneDistance(point,coe);
    }

} // namespace 