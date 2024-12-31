#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include "interfaces/srv/imagerequest.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <thread>

using namespace std::chrono;
using namespace std::placeholders;

typedef std::vector<std::vector<cv::Point>> Counters;
typedef std::vector<cv::Point> Counter;

std::mutex mtxvideoget;

const long double eps=1e-9;

class Arrow_detector:public rclcpp::Node{
    public:
    using Imagerequest=interfaces::srv::Imagerequest;
    Arrow_detector():Node("Arrow_detector"){
        this->client_=this->create_client<Imagerequest>("OriginalVideo");
        RCLCPP_INFO(this->get_logger(),"Arrow_detector client created !");
    }
    void CreatGetImageTimer();
    void GetImage();
    private:
    rclcpp::Client<Imagerequest>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
    // void MainArrowDetector(const sensor_msgs::msg::Image::SharedPtr msg);
    rclcpp::Node::SharedPtr node_shred_ptr;
    cv::Mat PreProgress(const cv::Mat & OriginalImage);
    bool MainDetectArrow(const cv::Mat & OriginalImage);
    bool TargetArrow(const cv::Mat & BinaryImage);
    cv::Mat OriginalImage;
};

void Arrow_detector::CreatGetImageTimer(){
    using namespace std::placeholders;
    using namespace std::chrono;
    node_shred_ptr=this->shared_from_this();
    // RCLCPP_INFO(this->get_logger(),"Start Creat Get Image Timer.");
    // timer_=this->create_wall_timer(33ms,std::bind(&Arrow_detector::GetImage,this));
    // RCLCPP_INFO(this->get_logger(),"Creat Get Image Timer.");
    // this->GetImage();
}

void Arrow_detector::GetImage(){
    using namespace std::chrono;
    mtxvideoget.lock();
    while(!client_->wait_for_service(1s)){
        if(!rclcpp::ok()){
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(this->get_logger(),"service not available, waiting again....");
    }

    auto request=std::make_shared<Imagerequest::Request>();
    auto result=client_->async_send_request(request);
    RCLCPP_INFO(this->get_logger(),"Send request.");
    sensor_msgs::msg::Image msg;

    if(rclcpp::spin_until_future_complete(node_shred_ptr,result)==rclcpp::FutureReturnCode::SUCCESS){
    // if(result.wait_for(1s)!=std::future_status::timeout){
        RCLCPP_INFO(this->get_logger(),"!!!!!");
        auto result_=result.get();
        if(result_->end){
            RCLCPP_INFO(this->get_logger(),"Video comes to end.");
            return;
        }
        msg=result_->image;
        RCLCPP_INFO(this->get_logger(),"Get frame with size [%d,%d]",msg.height,msg.width);
    }
    else {
        RCLCPP_ERROR(this->get_logger(), "Wait too long");
        return;
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

    mtxvideoget.unlock();
}

bool Arrow_detector::TargetArrow(const cv::Mat & BinaryImage){

    static int maxn=0,minn=1e9,approsiz=0;
    static bool pre=0;
    cv::Mat CounterImage;
    Counters counters_;
    Counter isarrow;

    cv::Canny(BinaryImage,CounterImage,100.,200.,3,true);
    
    // cv::findContours(BinaryImage,counters_,cv::RETR_LIST,cv::CHAIN_APPROX_SIMPLE);

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

        bool pixel_in=(pixel_num>=20000&&pixel_num<=80000);
        bool lwratio=1.1<=LengthWidthRatio&&LengthWidthRatio<=(4/sqrt(2));
        bool approxsize=(6<=approxcurve.size()&&approxcurve.size()<=10);

        if(lwratio&&pixel_in){
            maxn=std::max(maxn,pixel_num);
            minn=std::min(minn,pixel_num);
            
            RCLCPP_INFO(this->get_logger(),"size of approxcurve: %ld",approxcurve.size());
            RCLCPP_INFO(this->get_logger(),"max min pixel %d,%d",maxn,minn);
            cv::drawContours(OriginalImage,Counters{approxcurve},-1,cv::Scalar(0,225,0),1);
            cv::imshow("approxcurve",OriginalImage);
            cv::waitKey(33);
            pre=1;
        }
        else{
            pre=0;
            cv::drawContours(OriginalImage,Counters{approxcurve},-1,cv::Scalar(0,225,0),1);
            cv::imshow("approxcurve",OriginalImage);
            cv::waitKey(0);
        }

    }

    return 1;
}

bool Arrow_detector::MainDetectArrow(const cv::Mat & OriginalImage){
    cv::Mat Binary=PreProgress(OriginalImage);

    bool HaveArrow=TargetArrow(Binary);
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
    minval=std::max(100.0,minval);
    maxval=std::max(minval,maxval);

    RCLCPP_INFO(this->get_logger(),"maxval of greyimage : %lf ,minval of greyimage : %lf ",maxval,minval);

    cv::threshold(GreyImage,BinaryImage,maxval*threshholdk,300,cv::THRESH_BINARY);

    cv::dilate(BinaryImage,DilatedImage,cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(3,3)));

    cv::imshow("Pre",BinaryImage);
    // cv::imshow("Ori",OriginalImage);
    // cv::imshow("Gre",GreyImage);
    cv::imshow("Dilated",DilatedImage);
    cv::waitKey(33);
    return DilatedImage;
}

int main (int argc,char* argv[]){
    rclcpp::init(argc,argv);
    auto node=std::make_shared<Arrow_detector>();
    node->CreatGetImageTimer();
    // rclcpp::TimerBase::SharedPtr timer_=node->create_wall_timer(33ms,std::bind(&Arrow_detector::GetImage,node));

    while(1){
        // std::this_thread::sleep_for(33ms);
        node->GetImage();
    }

    rclcpp::spin(node);
    rclcpp::shutdown();
}