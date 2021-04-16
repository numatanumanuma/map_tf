#include "map_tf/map_tf.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "map_tf");
  //ノード名の初期化

  mapTF mapTF;

  ros::spin();

  return 0;
}