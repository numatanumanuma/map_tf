#ifndef MAP_TF_H
#define MAP_TF_H

#include "ros/ros.h"
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>
#include <opencv2/opencv.hpp>
   
class mapTF{
public:

    struct Point2D{
        double x;
        double y;
        double yaw;
        int x_p;
        int y_p;
        double yaw_p;
    };

    mapTF();
    mapTF(std::string map_topic);
    ~mapTF();
    void msgsCallback(const nav_msgs::OccupancyGrid::ConstPtr& msgs);
    void timerCallback(const ros::TimerEvent&);
    void getNowPoint(Point2D& p);
    void pointToPixel(Point2D& p);
    int width;
    int height;
    double resolution;
    bool get_map;
    cv::Mat image;
    cv::Mat3b dot_image;

    void printNowPoint();
   
private:
    ros::Subscriber map_sub_;
    ros::Timer timer_;
    ros::NodeHandle nh_;
    tf::TransformListener tf_listener_;
    std::string image_path_;
    std::string map_topic_;

};

#endif
