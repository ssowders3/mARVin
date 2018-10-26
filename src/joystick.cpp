#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Joy.h"

#include <sstream>

class ReadJoystick {
public:
  ReadJoystick();
private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  ros::NodeHandle nh_;

  int linear_, angular_;
  double l_scale_, a_scale_;
  ros::Subscriber joy_sub_;
  ros::Publisher out_data_;
};

ReadJoystick::ReadJoystick():
  linear_(1),
  angular_(2)
{
  nh_.param("axis_linear", linear_, linear_);
  nh_.param("axis_angular", angular_, angular_);
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);

  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &ReadJoystick::joyCallback,
   this);
  out_data_ = nh_.advertise<std_msgs::String>("chatter", 1000);
}

void ReadJoystick::joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
  double a = a_scale_*joy->axes[angular_];
  double x = l_scale_*joy->axes[linear_];

  std_msgs::String msg;
  std::stringstream ss;
  ss << "hello world " << a << x;
  msg.data = ss.str();
  ROS_INFO("%s", msg.data.c_str());
  out_data_.publish(msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "joystick");
  ReadJoystick read_joystick;

  ros::spin();

  /*ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }*/


  return 0;
}