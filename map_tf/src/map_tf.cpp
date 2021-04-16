#include "map_tf/map_tf.h"
   
mapTF::mapTF(){
    ros::NodeHandle nh_private("~");
    map_sub = nh.subscribe("/map", 1, &mapTF::msgsCallback, this);
    timer = nh.createTimer(ros::Duration(0.05), &mapTF::timerCallback, this);
    getMap = false;
    //mapをパブリッシュしたかどうかのフラグ
}

mapTF::~mapTF(){

}

void mapTF::msgsCallback(const nav_msgs::OccupancyGrid::ConstPtr& msgs){
    ROS_INFO("width:%d",msgs->info.width);
    ROS_INFO("height:%d",msgs->info.height);
    ROS_INFO("resolution:%f",msgs->info.resolution);
    width = msgs->info.width;
    height = msgs->info.height;
    resolution = msgs->info.resolution;

    getMap = true;
}

void mapTF::timerCallback(const ros::TimerEvent&){
    printNowpoint();
}

Point2D mapTF::getnowPoint(){
    //現在地を取得
    //中心が原点の座標系(m, rad)
    Point2D tmp_point;

    try{
        tf::StampedTransform trans;
        tf_listener.waitForTransform("map", "base_link", 
                ros::Time(0), ros::Duration(0.5));
        // 待つやつ この時間より長くは待たない
        tf_listener.lookupTransform("map", "base_link", 
                ros::Time(0), trans);
        // map座標->base_linkで最新(Time(0))をtransに受け取る
        // /mapからみた/base_link
        tmp_point.x = trans.getOrigin().x();
        tmp_point.y = trans.getOrigin().y();
        tmp_point.yaw = tf::getYaw(trans.getRotation());
    }catch(tf::TransformException &e){
        ROS_WARN("%s", e.what());
    }
    return tmp_point;
}

Point2D_pixel mapTF::pointToPixel(const Point2D& p){
    // map座標を(m)から(pixel)に変換また原点を左上の座標系にする
    Point2D_pixel tmppoint;
    tmppoint.x = (p.x + width / 2 * resolution) / resolution;
    tmppoint.y = (p.y - height / 2 * resolution) / resolution * -1;
    tmppoint.yaw = p.yaw * -1;
    return tmppoint;
}

void mapTF::printNowpoint(){
    // 現在地を出力
    Point2D nowpoint = getnowPoint();
    Point2D_pixel nowPoint2D_pixel = pointToPixel(nowpoint);
    ROS_INFO("%lf, %lf\n%d, %d", nowpoint.x, nowpoint.y, nowPoint2D_pixel.x, nowPoint2D_pixel.y);
}
