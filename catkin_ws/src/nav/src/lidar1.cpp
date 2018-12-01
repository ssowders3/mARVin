#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Range.h"
#include <iostream>

ros::Publisher range_pub;

void chatterCallback(const std_msgs::String::ConstPtr& msg) {
  std::string str = msg->data;
  std::string dist_str = str.substr(4);
  int dist = std::stoi(dist_str, nullptr, 10);
  float dist_meters = (float) dist/100.0;

  sensor_msgs::Range range_msg;
  range_msg.radiation_type = sensor_msgs::Range::INFRARED;
  range_msg.header.frame_id = "lidar1";
  range_msg.field_of_view = 0.1;
  range_msg.min_range = 0.3;
  range_msg.max_range = 12;
  range_msg.range = dist_meters;

  range_pub.publish(range_msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "lidar1");
  ros::NodeHandle n;
  range_pub = n.advertise<sensor_msgs::Range>("lidar1", 50);

  ros::Subscriber sub = n.subscribe("ld1", 1000, chatterCallback);
  ros::spin();
  return 0;
}
