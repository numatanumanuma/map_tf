#include "map_tf/map_tf.h"

mapTF::mapTF(){
    map_sub_ = nh_.subscribe("/map", 1, &mapTF::msgsCallback, this);
    timer_ = nh_.createTimer(ros::Duration(0.5), &mapTF::timerCallback, this);
    get_map = false;
}
   
mapTF::mapTF(std::string map_topic){
    map_sub_ = nh_.subscribe(map_topic, 1, &mapTF::msgsCallback, this);
    timer_ = nh_.createTimer(ros::Duration(0.5), &mapTF::timerCallback, this);
    get_map = false;
}

mapTF::~mapTF(){

}

void mapTF::msgsCallback(const nav_msgs::OccupancyGrid::ConstPtr& map){
    width = map->info.width;
    height = map->info.height;
    resolution = map->info.resolution;
    ROS_INFO("width:%d", width);
    ROS_INFO("height:%d", height);
    ROS_INFO("resolution:%f", resolution);

    image = cv::Mat::zeros(cv::Size(width,height), CV_8UC1);

    for (unsigned int y = 0; y < height; y++){
        for (unsigned int x = 0; x < width; x++){
            unsigned int i = x + (height - y - 1) * width;
            int intensity=205;
            if (map->data[i] >= 0 && map->data[i] <=100)
            intensity= round((float)(100.0-map->data[i])*2.55);
            image.at<unsigned char>(y, x)=intensity;
        }
    }
    cv::cvtColor(image, image, CV_GRAY2BGR);
    get_map = true;
}

void mapTF::timerCallback(const ros::TimerEvent&){
    // printNowpoint();
}

/*
 現在地を取得
 中心が原点の座標系(m, rad)
*/
void mapTF::getNowPoint(Point2D& p){ 
    try{
        tf::StampedTransform trans;
        tf_listener_.waitForTransform("map", "base_link", 
                ros::Time(0), ros::Duration(0.5));
        // 待つやつ この時間より長くは待たない
        tf_listener_.lookupTransform("map", "base_link", 
                ros::Time(0), trans);
        // map座標->base_linkで最新(Time(0))をtransに受け取る
        // /mapからみた/base_link
        p.x = trans.getOrigin().x();
        p.y = trans.getOrigin().y();
        p.yaw = tf::getYaw(trans.getRotation());
        pointToPixel(p);
    }catch(tf::TransformException &e){
        ROS_WARN("%s", e.what());
    }
    return;
}

/* map座標を(m)から(pixel)に変換また原点を左上の座標系にする */
void mapTF::pointToPixel(Point2D& p){
    p.x_p = (p.x + width / 2 * resolution) / resolution;
    p.y_p = (p.y - height / 2 * resolution) / resolution * -1;
    p.yaw_p = p.yaw * -1;
    return;
}

/* 現在地を出力 */
void mapTF::printNowPoint(){
    Point2D p;
    getNowPoint(p);
    ROS_INFO("Now...%lf, %lf\n%d, %d", p.x, p.y, p.x_p, p.y_p);
    return;
}
