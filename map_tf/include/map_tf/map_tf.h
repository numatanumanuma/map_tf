#ifndef MAP_TF_H
#define MAP_TF_H

#include "ros/ros.h"
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>

#include "map_tf/Point2D.h"
#include "map_tf/Point2D_pixel.h"
   
class mapTF{
public:
    mapTF();
    ~mapTF();
    void msgsCallback(const nav_msgs::OccupancyGrid::ConstPtr& msgs);
    void timerCallback(const ros::TimerEvent&);
    Point2D getnowPoint();
    Point2D_pixel pointToPixel(const Point2D& p);
    //参照渡し const
    void printNowpoint();
   
private:
    ros::Subscriber map_sub;
    ros::Timer timer;
    ros::NodeHandle nh;
    tf::TransformListener tf_listener;
    int width;
    int height;
    double resolution;
    bool getMap;
};

#endif
