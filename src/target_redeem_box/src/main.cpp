#include "target_redeem_box/RedeemBox_detector.hpp"

// this file place some base class function

namespace Engineering_robot_RM2025_Pnx{


bool RedeemBox_detector::PnPsolver(const std::vector<cv::Point2d > & ImagePoints2D,const std::vector<cv::Point3d > & ObjectPoints3D,const std::vector<double> & cameraMatrix,const std::vector<double> & distCoeffs,
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

    //check pnp success

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

    RCLCPP_INFO(this->get_logger(),"PnPsolver finish");
    return 1;
}

void RedeemBox_detector::DrawPnPResult(cv::Mat &Image, const cv::Mat & rvec, const cv::Mat & tvec, cv::Scalar color, int thickness, cv::Point textpos){
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
    Eigen::Matrix<double,4,1> Vectorz3D=rtvecEigen*Eigen::Matrix<double,4,1>(0,0,1,1);
    Center3D/=Center3D(3);
    Vectorz3D/=Vectorz3D(3);
    Eigen::Matrix<double,3,1> Center2D=cameraMatrixEigen*signMat*Center3D;
    Eigen::Matrix<double,3,1> CenterVectorz2D=cameraMatrixEigen*signMat*Vectorz3D;
    CenterVectorz2D/=CenterVectorz2D(2);
    Center2D/=Center2D(2);

    std::vector<cv::Point> reput_arrow;

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

void RedeemBox_detector::SendBoxPosition(cv::Mat & tvec,cv::Mat & rvecmat, bool reverse){

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

    geometry_msgs::msg::TransformStamped box_to_camera;

    cv::Vec4d Quaternion_r=rotationMatrixToQuaternion(rmat);

    box_to_camera.header.stamp=this->now();
    box_to_camera.header.frame_id=ImageFrame;
    box_to_camera.child_frame_id="object/box";
    box_to_camera.transform.translation.x=tvec.at<double>(0);
    box_to_camera.transform.translation.y=tvec.at<double>(1);
    box_to_camera.transform.translation.z=tvec.at<double>(2);
    box_to_camera.transform.rotation.w=Quaternion_r[0];
    box_to_camera.transform.rotation.x=Quaternion_r[1];
    box_to_camera.transform.rotation.y=Quaternion_r[2];
    box_to_camera.transform.rotation.z=Quaternion_r[3];

    if(reverse){
        box_to_camera=ReverseTransforme(box_to_camera);
    }

    tf_broadcaster_box_to_camera->sendTransform(box_to_camera);

    RCLCPP_INFO(this->get_logger(),"tf_broadcaster_box_to_camera pub successfully");
}

void RedeemBox_detector::SyncPubBoxPos(){
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
    
void RedeemBox_detector::SyncPubBoxPosInit(){
    YAML::Node syncconfig=config["arrow_detect"]["SyncPubBoxPos"];
    queuesiz=syncconfig["queuesiz"].as<int>();
    syncThresehold=rclcpp::Duration(syncconfig["syncThresehold"].as<std::vector<int>>()[0],
        syncconfig["syncThresehold"].as<std::vector<int>>()[1]);

    respubtimer_=this->create_wall_timer(std::chrono::milliseconds(syncconfig["PubInterval"].as<int>()),std::bind(&RedeemBox_detector::SyncPubBoxPos,this));
}

void RedeemBox_detector::CallDetectorFunctions(){
    // 获取图像拷贝
    cv::Mat imageCopy;
    {
        std::lock_guard<std::mutex> lock(OriginalImage_mutex);
        if(OriginalImage.empty() || OriginalImage.size()==cv::Size(0,0)){
            RCLCPP_WARN(this->get_logger(),"OriginalImage is empty, can't detect.");
            return;
        }
        imageCopy = OriginalImage.clone();
    }

    // 显示图像
    try {
        cv::imshow("Original", imageCopy);
        cv::waitKey(1);
    } catch (const cv::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "OpenCV display error: %s", e.what());
        return;
    }

    // 准备线程数据
    struct callTime{
        std::chrono::_V2::system_clock::time_point begin;
        std::chrono::_V2::system_clock::time_point end;
        std::string name;
        bool success = false;
    };
    std::vector<callTime> times(callback_functions.size());
    std::vector<std::thread> threads;
    std::mutex times_mutex;

    // 创建并运行线程
    for(std::size_t i=0; i<callback_functions.size(); i++){
        try {
            if(!callback_functions[i]) {
                RCLCPP_WARN(this->get_logger(), "Callback function %s is null", callback_functions_names[i].c_str());
                continue;
            }

            threads.emplace_back([i, &times, &times_mutex, this, imageCopy](){
                callTime ct;
                ct.begin = std::chrono::high_resolution_clock::now();
                ct.name = callback_functions_names[i];
                
                try {
                    if(!imageCopy.empty()) {
                        callback_functions[i](imageCopy);
                        ct.success = true;
                    } else {
                        RCLCPP_WARN(this->get_logger(),"Skipping %s - empty image", ct.name.c_str());
                    }
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(),"Error in %s: %s", ct.name.c_str(), e.what());
                }

                ct.end = std::chrono::high_resolution_clock::now();
                
                {
                    std::lock_guard<std::mutex> lock(times_mutex);
                    times[i] = ct;
                }
            });
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(),"Failed to create thread for %s: %s", 
                        callback_functions_names[i].c_str(), e.what());
        }
    }

    // 等待所有线程完成
    for(auto& t : threads) {
        if(t.joinable()) {
            try {
                t.join();
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(),"Thread join error: %s", e.what());
            }
        }
    }

    // 记录执行时间
    for(const auto& ct : times) {
        if(ct.name.empty()) continue; // 跳过无效记录
        
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(ct.end - ct.begin);
        if(ct.success) {
            RCLCPP_INFO(this->get_logger(),"%s completed in %ld ms", ct.name.c_str(), duration.count());
        } else {
            RCLCPP_WARN(this->get_logger(),"%s failed after %ld ms", ct.name.c_str(), duration.count());
        }
    }
}


void RedeemBox_detector::GetImage(const sensor_msgs::msg::Image::SharedPtr msg){
    std::chrono::_V2::system_clock::time_point GetImageBegine=std::chrono::high_resolution_clock::now();
    while(OriginalImage_mutex.try_lock()==false){
        if((std::chrono::high_resolution_clock::now()-GetImageBegine)>std::chrono::milliseconds(10)){
            RCLCPP_WARN(this->get_logger(),"ImageClinentHandle : GetImage timeout");
            return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    try{
        OriginalImage=std::move(cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8)->image);
    }
    catch(cv_bridge::Exception& e){
        RCLCPP_ERROR(this->get_logger(),"ImageClinentHandle : cv_bridge exception : %s",e.what());
        OriginalImage_mutex.unlock();
        return;
    }

    OriginalImage_mutex.unlock();
}

void RedeemBox_detector::CloudSubManage(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg){
    sensor_msgs::msg::PointCloud2 TransformedCloudPoint;
    geometry_msgs::msg::TransformStamped transform;
    std::pair<int,rclcpp::Time> tmp=std::make_pair(cloud_msg->width*cloud_msg->height,cloud_msg->header.stamp);
    // pcl::PointCloud<pcl::PointXYZ> InputCloud;

    try{
        transform=tf2_buffer_->lookupTransform(
            ImageFrame,
            cloud_msg->header.frame_id,
            this->now(),
            rclcpp::Duration(1,0)
            );

        #ifdef test_pcl_manage
        std::stringstream transformsss;
        transformsss<<"rotation: "<<
        transform.transform.rotation.x<<" "<<
        transform.transform.rotation.y<<" "<<
        transform.transform.rotation.z<<" "<<
        transform.transform.rotation.w<<"\n";
        transformsss<<"translation: "<<
        transform.transform.translation.x<<" "<<
        transform.transform.translation.y<<" "<<
        transform.transform.translation.z;
        RCLCPP_INFO(this->get_logger(),"transform %s",transformsss.str().c_str());
        #endif
    }
    catch (tf2::TransformException &ex){
        RCLCPP_WARN(this->get_logger(),"[ImageCloudPointCallBack]: %s",ex.what());
        return;
    }
    tf2::doTransform(*cloud_msg,TransformedCloudPoint,transform);

    Cloudmtx.lock();
    
    pcl::fromROSMsg(TransformedCloudPoint,InTimeCloud);

    # ifdef test_pointcloud_main_log

    RCLCPP_INFO(this->get_logger(),"[CloudSubManage] : now point cloud size %d", 
        InTimeCloud.size());

    #endif

    Cloudmtx.unlock();
}

void RedeemBox_detector::InTimeCloudUpdate(){
    std::lock_guard<std::mutex> lock_guarde(Cloudmtx);
    while(!CloudTimeStamp.empty()&&(this->now()-CloudTimeStamp.front().second)>buffertime){
        InTimeCloud.erase(InTimeCloud.begin(),InTimeCloud.begin()+CloudTimeStamp.front().first);
      this->CloudTimeStamp.pop();
    }
}


RedeemBox_detector::RedeemBox_detector(rclcpp::NodeOptions options):
    Node("RedeemBox_detector",options){

    OriginalImage=cv::Mat::zeros(cv::Size(0,0),CV_8UC1);

    this->declare_parameter<std::string>("Location","/home/pnx/code/Engineering_robot_RM2025_Pnx/");
    
    this->declare_parameter<std::string>("CloudPointTopic","sensor/RealSense/point_cloud");
    this->declare_parameter<std::string>("ImageTopic","sensor/camera/images");
    this->declare_parameter<std::string>("CloudPointFrame","sensor/RealSense/depth");
    this->declare_parameter<std::string>("ImageFrame","sensor/camera");

    CloudPointTopic=this->get_parameter("CloudPointTopic").as_string();
    ImageTopic=this->get_parameter("ImageTopic").as_string();
    CloudPointFrame=this->get_parameter("CloudPointFrame").as_string();
    ImageFrame=this->get_parameter("ImageFrame").as_string();

    RCLCPP_INFO_STREAM(this->get_logger(),"CloudPointTopic : "<<CloudPointTopic);
    RCLCPP_INFO_STREAM(this->get_logger(),"ImageTopic : "<<ImageTopic);
    RCLCPP_INFO_STREAM(this->get_logger(),"CloudPointFrame : "<<CloudPointFrame);
    RCLCPP_INFO_STREAM(this->get_logger(),"ImageFrame : "<<ImageFrame);

    tf2_buffer_=std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf2_listener_=std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_,this);

    try{
        config = YAML::LoadFile(this->get_parameter("Location").as_string()+"/src/config.yaml");
        

        //ArrowDetector part

        YAML::Node configDetectArrowInit;
        try{
            configDetectArrowInit=config["RedeemBox_detector"]["Parameters"];
    
            for(int i=0;i<8;i++){
                const std::vector<double> & arrowPoints=config["RedeemBox_detector"]["KeyPoints"]["arrow"]["arrowPoints"][i].as<std::vector<double>>();
                objpoints.push_back(cv::Point3d(arrowPoints[0],arrowPoints[1],arrowPoints[2]));
                objpointsEigen.push_back(Eigen::Vector4d(arrowPoints[0],arrowPoints[1],arrowPoints[2],1));
            }
            ArrowDetectorThresholdThresh=configDetectArrowInit["arrow_detect"]["ArrowDetectorThresholdThresh"].as<double>();
            ArrowDetectorThresholdMaxval=configDetectArrowInit["arrow_detect"]["ArrowDetectorThresholdMaxval"].as<double>();    
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
            ArrowDetectorThresholdThreshold=configDetectArrowInit["arrow_detect"]["ArrowDetectorThresholdThreshold"].as<double>();
            ArrowDetectorIterations=configDetectArrowInit["arrow_detect"]["ArrowDetectorIterations"].as<double>();
            ArrowDetectorapproxPolyDPEpsilon=configDetectArrowInit["arrow_detect"]["ArrowDetectorapproxPolyDPEpsilon"].as<double>();
            ArrowDetectorLongShortRateMax=configDetectArrowInit["arrow_detect"]["ArrowDetectorLongShortRateMax"].as<double>();
            ArrowDetectorLongShortRateMin=configDetectArrowInit["arrow_detect"]["ArrowDetectorLongShortRateMin"].as<double>();
    
        }
        catch(const std::exception& e){
            RCLCPP_ERROR(this->get_logger(),"DetectArrowInit failed - %s",e.what());
            rclcpp::shutdown();
        }

        ImageWidth=config["camera"]["width"].as<int>();
        ImageHeight=config["camera"]["height"].as<int>();
        cameraMatrix=config["camera"]["camera_matrix"].as<std::vector<double>>();
        distCoeffs=config["camera"]["dist_coeffs"].as<std::vector<double>>();
        cameraMatrixMat=cv::Mat(cv::Size(3,3),CV_64F);
        for(int i=0;i<9;i++){
            cameraMatrixEigen(i/3,i%3)=cameraMatrix[i];
            cameraMatrixMat.at<double>(i/3,i%3)=cameraMatrix[i];
        }
        InverseCameraMatrixEigen=cameraMatrixEigen.inverse();

        for(int i=0;i<4;i++){
            const std::vector<double> & redeemptionBoxCornerPoints=config["RedeemBox_detector"]["KeyPoints"]["redeem_box"]["redeemptionBoxCornerPoints"][i].as<std::vector<double>>();
            ObjRedemptionBoxCornerPoint.push_back(cv::Point3d(redeemptionBoxCornerPoints[0],redeemptionBoxCornerPoints[1],redeemptionBoxCornerPoints[2]));
            ObjRedemptionBoxCornerPointEigen.push_back(Eigen::Vector4d(redeemptionBoxCornerPoints[0],redeemptionBoxCornerPoints[1],redeemptionBoxCornerPoints[2],1));
        }
        for(int i=0;i<2;i++){
            const std::vector<double> & line=config["RedeemBox_detector"]["KeyPoints"]["redeem_box"]["line"][i].as<std::vector<double>>();
            Object2cornersEigen.push_back(Eigen::Vector4d(line[0],line[1],line[2],1));
        }
        frontfacecenter=Eigen::Matrix<double,4,1>(config["RedeemBox_detector"]["KeyPoints"]["redeem_box"]["center"][0].as<double>(),
            config["RedeemBox_detector"]["KeyPoints"]["redeem_box"]["center"][1].as<double>(),
            config["RedeemBox_detector"]["KeyPoints"]["redeem_box"]["center"][2].as<double>(),
            1.0
        );

        buffertime=rclcpp::Duration(config["RedeemBox_detector"]["Parameters"]["Main"]["buffertime"].as<std::vector<int>>()[0],config["RedeemBox_detector"]["Parameters"]["Main"]["buffertime"].as<std::vector<int>>()[1]) ;

    }
    catch(const std::exception& e){
        RCLCPP_ERROR(this->get_logger(),"Fail to load config file : %s",e.what());
        rclcpp::shutdown();
    }

    signMat<<1,0,0,0,
        0,1,0,0,
        0,0,1,0;
    
    //定义参数
    try{

        tf_broadcaster_box_to_camera=std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // static_tf_broadcaster_camera_to_arm=std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);


        // geometry_msgs::msg::TransformStamped camera_to_arm;

        // camera_to_arm.header.frame_id="map";
        // camera_to_arm.child_frame_id="object/arm";
        // camera_to_arm.transform.translation.x=config["object_pos"]["arm"]["translation"]["x"].as<double>();
        // camera_to_arm.transform.translation.y=config["object_pos"]["arm"]["translation"]["y"].as<double>();
        // camera_to_arm.transform.translation.z=config["object_pos"]["arm"]["translation"]["z"].as<double>();
        // camera_to_arm.transform.rotation.w=config["object_pos"]["arm"]["rotate"]["w"].as<double>();
        // camera_to_arm.transform.rotation.x=config["object_pos"]["arm"]["rotate"]["x"].as<double>();
        // camera_to_arm.transform.rotation.y=config["object_pos"]["arm"]["rotate"]["y"].as<double>();
        // camera_to_arm.transform.rotation.z=config["object_pos"]["arm"]["rotate"]["z"].as<double>();

        // static_tf_broadcaster_camera_to_arm->sendTransform(camera_to_arm);

    }
    catch(const std::exception& e){
        RCLCPP_ERROR(this->get_logger(),"Fail to load config file : %s",e.what());
        rclcpp::shutdown();
    }

    RCLCPP_INFO(this->get_logger(),"MainInit finish, start function laod");

    // this->image_client_=this->create_client<interfaces::srv::Imagerequest>("/sensor/camera/images");
    this->label_image_pub_=this->create_publisher<sensor_msgs::msg::Image>("/arrow_detect/label_image",10);
    this->Image_sub_=this->create_subscription<sensor_msgs::msg::Image>(ImageTopic,10,std::bind(&RedeemBox_detector::GetImage,this,_1));
    this->cloud_sub_=this->create_subscription<sensor_msgs::msg::PointCloud2>(CloudPointTopic,
        10,
        std::bind(&RedeemBox_detector::CloudSubManage, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(),"RedeemBox_detector client created !");

    bool detectArrowInitialized = false;
    bool rectangleDetectorInitialized = false;
    bool pclManageInitialized = false;

    if(config["RedeemBox_detector"]["LaunchMode"]["DetectArrow"].as<bool>()){
        try {
            DetectArrowInit();
            callback_functions.push_back(std::bind(&RedeemBox_detector::MainDetectArrow,this,_1));
            callback_functions_names.push_back("DetectArrow");
            detectArrowInitialized = true;
            RCLCPP_INFO(this->get_logger(),"DetectArrow module initialized successfully");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(),"Failed to initialize DetectArrow: %s", e.what());
        }
    }

    if(config["RedeemBox_detector"]["LaunchMode"]["TargetRectangle"].as<bool>()){
        try {
            RectangleDetectorInit();
            callback_functions.push_back(std::bind(&RedeemBox_detector::MainDetectArrow_Rectangle,this,_1));
            callback_functions_names.push_back("TargetRectangle");
            rectangleDetectorInitialized = true;
            RCLCPP_INFO(this->get_logger(),"TargetRectangle module initialized successfully");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(),"Failed to initialize TargetRectangle: %s", e.what());
        }
    }

    if(config["RedeemBox_detector"]["LaunchMode"]["PclManage"].as<bool>()){
        try {
            PointCloudeInit();
            callback_functions.push_back(std::bind(&RedeemBox_detector::MainPclManager,this,_1));
            callback_functions_names.push_back("PclManage");
            pclManageInitialized = true;
            RCLCPP_INFO(this->get_logger(),"PclManage module initialized successfully");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(),"Failed to initialize PclManage: %s", e.what());
        }
    }

    if(callback_functions.empty()) {
        RCLCPP_ERROR(this->get_logger(),"No detector modules initialized successfully, shutting down");
        rclcpp::shutdown();
    }

    RCLCPP_INFO(this->get_logger(), "%ld Functions load completed",callback_functions.size());
    for(auto & i : callback_functions_names){
        RCLCPP_INFO(this->get_logger(),"function name : %s",i.c_str());
    }
    

    #ifdef SyncPubBoxPos
    SyncPubBoxPosInit();
    #endif

    try{
        ImageProcessorThread=std::thread([this](){
            while(1){
                CallDetectorFunctions();
                std::this_thread::sleep_for(10ms);
                if(!rclcpp::ok()){
                    rclcpp::shutdown();
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(3));
            }
        });
    }
    catch(const std::exception& e){
        RCLCPP_ERROR(this->get_logger(),"Fail to create ImageProcessorThread : %s",e.what());
        rclcpp::shutdown();
    }
    RCLCPP_INFO(this->get_logger(),"MainLoop finish");
}

} // namespace Engineering_robot_RM2025_Pnx

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(Engineering_robot_RM2025_Pnx::RedeemBox_detector)
