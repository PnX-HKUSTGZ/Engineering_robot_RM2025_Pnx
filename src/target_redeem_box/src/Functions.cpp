#include "RedeemBox_detector.hpp"

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

} // namespace 