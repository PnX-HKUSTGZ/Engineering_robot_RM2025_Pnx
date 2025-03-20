#include "detector.hpp"

/*
由于没有相机，所以没有对于画面的畸变矫正

这里完成了通道分离，二值化

注意坑在于 split mixChannels merge 函数
*/
cv::Mat preprocess_image_exchange(const cv::Mat& image){
    Splited_images channels;
    Mat image_binary;
    //通道顺序为 BGR
    cv::split(image,channels);
    Mat image_BR(channels[0].size(),channels[0].type());

    const vector<int> from_to={
        0,0,
        1,0
    };

    cv::mixChannels(vector<cv::Mat>{channels[0],channels[2]},vector<cv::Mat>{image_BR},from_to);

    cv::threshold(image_BR,image_binary,100,200,cv::THRESH_BINARY);

    // std::cout<<image_BR.channels()<<std::endl;

    // cv::namedWindow("1");
    // cv::namedWindow("2");
    // cv::namedWindow("3");
    // cv::imshow("1",image);
    // cv::imshow("2",image_BR);
    // cv::imshow("3",image_binary);
    // cv::waitKey(0);
    return  image_binary;
}

bool find_front_exchange_slot(const Mat& image_pre,Mat& image){
    Counters counter_1,counter_2,counter_3;
    vector<cv::RotatedRect> counter_3rects;
    vector<int> counter_size;
    cv::findContours(image_pre,counter_1,cv::RETR_LIST,cv::CHAIN_APPROX_SIMPLE);
    Counter result;
    cv::RotatedRect rect;
    cv::Size2f siz;
    // cv::drawContours(image,counter_1,-1,blue,5);
    // cv::imshow("1",image);
    // cv::waitKey(0);
    for(const Counter &counter : counter_1){
        // auto I=image.clone();
        // cv::drawContours(I,Counters{counter},-1,blue,5);
        // cv::imshow("1",I);
        // cv::waitKey(0);
        //这里的epsilon的值是没有确定的，要看之后怎么调；
        cv::approxPolyDP(counter,result,10,1);
        rect=cv::minAreaRect(counter);
        siz=rect.size;
        int pixel_num=cv::contourArea(counter);
        //参数全部等待确定，和相机与工作距离有关
        bool height_to_width=(siz.width>eps)&&(siz.height>eps)&&(siz.height/siz.width<4.5 && siz.width/siz.height<4.5);
        // bool siz_counter=1;
        bool siz_counter=(pixel_num>400 && pixel_num<6000);
        bool approxPolyDP_counters=(result.size()>=4 && result.size()<9);
        if(height_to_width&&siz_counter&&approxPolyDP_counters) counter_2.push_back(counter),counter_size.push_back(pixel_num),cout<<1<<endl;
        else counter_3.push_back(counter),counter_3rects.push_back(rect);
    }
    sort(counter_size.begin(),counter_size.end());
    if(counter_2.size()!=4||(counter_size[0]*10<=counter_size[3])) return 0;
    //找到了四个角

    // cv::drawContours(image,counter_2,-1,blue,5);
    // cv::imshow("1",image);
    // cv::waitKey(0);

    Circles circles(4);
    Counters trangles(4);
    Counter rectangle;
    cv::Point2f center;
    vector<float> angles;

    for(int i=0;i<4;i++){
        vector<cv::Point2f> add;
        for(int e=i-1;e<i+2;e++){
            //vector中的⻆点顺序永远为逆时针顺序
            for(const auto& point : counter_2[(e+4)%4]) add.push_back(point);
        }
        cv::minEnclosingCircle(add,circles[i].center,circles[i].radius);
        cv::minEnclosingTriangle(add,trangles[i]);

        // Mat Image=image.clone();
        // cv::drawContours(Image,Counters{trangles[i]},-1,blue,5);
        // cv::imshow("1",Image);
        // cv::waitKey(0);
    }

    for(auto & i: circles) center+=i.center;
    center/=4;

    // cv::circle(image,center,3,blue,-3);
    // cv::imshow("1",image);
    // cv::waitKey(0);
    
    for(const auto& trangle:trangles){
        // Mat Image=image.clone();
        // cv::drawContours(Image,Counters{trangle},-1,blue,5);
        // cv::imshow("1",Image);
        // cv::waitKey(0);
        bool get_across=0;
        angles=get_trangle_angle(trangle);
        for(int i=0;i<3;i++){
            if(angles[i]>=130){
                rectangle.push_back(trangle[i]);
                get_across=1;
                break;
            }
        }
        if(get_across) continue;

        float minn=1e9;int index=-1;
        for(int i=0;i<3;i++){
            // cv::circle(Image,trangle[i],3,green,-3);
            // cv::imshow("1",Image);
            // cv::waitKey(0);
            //判断逻辑还要改一下，不太清楚
            float dis=distanceBetweenPoint(trangle[i],center);
            if(dis<=minn) minn=dis,index=i;
        }
        rectangle.push_back(trangle[index]);
    }

    // cv::drawContours(image,Counters{rectangle},-1,blue,5);
    // cv::imshow("1",image);
    // cv::waitKey(0);

    //找完了四个角，开始排序

    // Counters small_rects;
    // int index_0;

    // for(int i=counter_3.size()-1;i>=0;i--){
    //     double siz_counter=cv::contourArea(counter_3[i]),siz_rect=counter_3rects[i].size.area();
    //     bool siz_factor=siz_counter>=50&&siz_counter<=200;
    //     bool siz_counter_to_siz_rect=siz_rect<=siz_counter*1.2;
    //     bool width_to_height=counter_3rects[i].size.height/counter_3rects[i].size.width<=2&&counter_3rects[i].size.width/counter_3rects[i].size.height<=2;
    //     if(siz_factor&&siz_counter_to_siz_rect&&width_to_height){
    //         small_rects.push_back(counter_3[i]);
    //     }
    // }

    // if(small_rects.empty()){
    //     for(auto & i : )
    // }

    //这里使用的是识别一个特殊的角来确定方向

    std::sort(rectangle.begin(),rectangle.end(),[&center](const cv::Point &x1,const cv::Point &x2){
        float angle1=angleBetweenVectors(cv::Point2f(1,0),cv::Point2f(x1)-center);
        float angle2=angleBetweenVectors(cv::Point2f(1,0),cv::Point2f(x2)-center);
        if(x1.y>=center.y) angle1=360-angle1;
        if(x2.y>=center.y) angle2=360-angle2;
        return angle1<angle2;
    });
    // 这个是按照顺时针存的，第一个角是当前图片中最左上角
    cv::drawContours(image,Counters{rectangle},-1,blue,5);
    cv::imshow("1",image);
    cv::waitKey(0);
    
    return 1;
}

void process_image_front(cv::Mat& image){
    cv::Mat image_binary=preprocess_image_exchange(image);
    find_front_exchange_slot(image_binary,image);
}

float angleBetweenVectors(const cv::Point2f& v1, const cv::Point2f& v2) {
    float cosAngle = v1.dot(v2) / (cv::norm(v1) * cv::norm(v2));
    return std::acos(cosAngle); // 返回的是弧度值
}

float distanceBetweenPoint(const cv::Point2f& v1, const cv::Point2f& v2){
    return sqrt((v1.x-v2.x)*(v1.x-v2.x)+(v1.y-v2.y)*(v1.y-v2.y));
}

vector<float> get_trangle_angle(const Counter& trangle){
    //存对边
    vector<float> angles;
    if(trangle.size()!=3){std::cerr<<"the trangle.size() is not 3\n";exit(0);}
    for(int i=0;i<3;i++){
        angles.push_back(angleBetweenVectors(trangle[(i+1)%3]-trangle[i],trangle[(i+2)%3]-trangle[i])*180.0/CV_PI);
    }
    return angles;
}