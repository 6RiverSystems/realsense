#include <ros/ros.h>

#include "ImageSubscriber.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ohps");

  ImageSubscriber imageSubscriber;
  ros::spin();

  return 0;
}
