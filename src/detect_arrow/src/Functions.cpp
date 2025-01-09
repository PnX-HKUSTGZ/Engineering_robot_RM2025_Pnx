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