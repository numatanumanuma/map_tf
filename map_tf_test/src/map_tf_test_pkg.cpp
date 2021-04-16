#include "ros/ros.h"
#include "map_tf/map_tf.h"


class mapTFTest{
  public:
    mapTFTest();
    ~mapTFTest();
    void timerCallback(const ros::TimerEvent&);
    mapTF maptf;
    // 宣言
  private:
    ros::NodeHandle nh;
    ros::Timer timer;
};

mapTFTest::mapTFTest(){
  timer = nh.createTimer(ros::Duration(0.1), &mapTFTest::timerCallback, this);
}

mapTFTest::~mapTFTest(){

}

void mapTFTest::timerCallback(const ros::TimerEvent&){
  maptf.printNowpoint();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "map_tf_test");
  //ノード名の初期化

  mapTFTest mapTFTest;

  ros::spin();

  return 0;
}
