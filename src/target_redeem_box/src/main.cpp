#include "RedeemBox_detector.hpp"

// this file place some base function

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
    DrawPnPResult(OriginalImage_Rectangle,rvecs[0],tvecs[0],cv::Scalar(225,0,0),2,cv::Point(20,40));
    DrawPnPResult(OriginalImage_Rectangle,rvecs[1],tvecs[1],cv::Scalar(100,0,200),2,cv::Point(20,100));
    DrawPnPResult(OriginalImage_Rectangle,rvec,tvec,cv::Scalar(100,100,200),2,cv::Point(20,160));
    auto lable_msg_ptr=cv_bridge::CvImage(std_msgs::msg::Header(),sensor_msgs::image_encodings::BGR8,OriginalImage_).toImageMsg();
    lable_msg_ptr->header.frame_id="/arrow_detect/label_image";
    lable_msg_ptr->header.stamp=this->get_clock()->now();
    this->label_image_pub_->publish(*lable_msg_ptr);

    cv::imshow("pnp result",OriginalImage_Rectangle);

    cv::waitKey(10);
    # endif

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

void RedeemBox_detector::SendBoxPosition(cv::Mat & tvec,cv::Mat & rvecmat,cv::Mat & OriginalImage){

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
    

bool RedeemBox_detector::MainDetectArrow(const cv::Mat & OriginalImage){
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

void RedeemBox_detector::SyncPubBoxPosInit(){
    YAML::Node syncconfig=config["arrow_detect"]["SyncPubBoxPos"];
    queuesiz=syncconfig["queuesiz"].as<int>();
    syncThresehold=rclcpp::Duration(syncconfig["syncThresehold"].as<std::vector<int>>()[0],
        syncconfig["syncThresehold"].as<std::vector<int>>()[1]);

    respubtimer_=this->create_wall_timer(std::chrono::milliseconds(syncconfig["PubInterval"].as<int>()),std::bind(&RedeemBox_detector::SyncPubBoxPos,this));
}



RedeemBox_detector::RedeemBox_detector(rclcpp::NodeOptions options):
    Node("RedeemBox_detector",options){
    // this->subscription_=this->create_subscription<sensor_msgs::msg::Image>("/sensor/image",10,std::bind(&RedeemBox_detector::GetImage,this,_1));
    this->image_client_=this->create_client<interfaces::srv::Imagerequest>("/sensor/camera/images",10,);
    this->label_image_pub_=this->create_publisher<sensor_msgs::msg::Image>("/arrow_detect/label_image",10);
    RCLCPP_INFO(this->get_logger(),"RedeemBox_detector client created !");

    this->declare_parameter<std::string>("Location","/home/lqx/code/Engineering_robot_RM2025_Pnx/");
    
    try{
        config = YAML::LoadFile(this->get_parameter("Location").as_string()+"/src/config.yaml");
        cameraMatrix=config["camera"]["camera_matrix"].as<std::vector<double>>();
        distCoeffs=config["camera"]["dist_coeffs"].as<std::vector<double>>();    
        for(int i=0;i<9;i++){
            cameraMatrixEigen(i/3,i%3)=cameraMatrix[i];
        }
        InverseCameraMatrixEigen=cameraMatrixEigen.inverse();

        for(int i=0;i<4;i++){
            const std::vector<double> & redeemptionBoxCornerPoints=config["redeem_box"]["redeemptionBoxCornerPoints"][i].as<std::vector<double>>();
            ObjRedemptionBoxCornerPoint.push_back(cv::Point3d(redeemptionBoxCornerPoints[0],redeemptionBoxCornerPoints[1],redeemptionBoxCornerPoints[2]));
            ObjRedemptionBoxCornerPointEigen.push_back(Eigen::Vector4d(redeemptionBoxCornerPoints[0],redeemptionBoxCornerPoints[1],redeemptionBoxCornerPoints[2],1));
        }
        for(int i=0;i<2;i++){
            const std::vector<double> & line=config["redeem_box"]["line"][i].as<std::vector<double>>();
            Object2cornersEigen.push_back(Eigen::Vector4d(line[0],line[1],line[2],1));
        }
        frontfacecenter=Eigen::Matrix<double,4,1>(config["redeem_box"]["center"][0].as<double>(),
            config["redeem_box"]["center"][1].as<double>(),
            config["redeem_box"]["center"][2].as<double>(),
            1.0
        );

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

    RCLCPP_INFO(this->get_logger(),"MainInit finish");

    #ifdef DetectorArrow
    DetectArrowInit();
    #endif

    #ifdef DetectorRectangle
    RectangleDetectorInit();
    #endif

    #ifdef PCLManager
    PointCloudeInit();
    #endif

    #ifdef SyncPubBoxPos
    SyncPubBoxPosInit();
    #endif
}
