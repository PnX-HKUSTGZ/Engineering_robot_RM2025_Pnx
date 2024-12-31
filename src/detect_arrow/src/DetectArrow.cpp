#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include "interfaces/srv/imagerequest.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <thread>

using namespace std::chrono;
using namespace std::placeholders;

std::mutex mtxvideoget;

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
    RCLCPP_INFO(this->get_logger(), "Get frame");
    // cv::imshow("Video",originalframe);
    // cv::waitKey(33);
    PreProgress(originalframe);
    mtxvideoget.unlock();
}

cv::Mat Arrow_detector::PreProgress(const cv::Mat & OriginalImage){

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