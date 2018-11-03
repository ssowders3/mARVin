// Start of Gps Node Program
// Inputs: Gps data from Android phone running ShareGPS
// Outputs: GPS Message of type sensor_msgs/NavSatFix.msg

#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/NavSatStatus.h"
#include <std_msgs/Float64.h>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <cstdio>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <array>
#include <thread>

sensor_msgs::NavSatStatus navStatus;
float latitude, longitude, altitude;
float position_covariance[9] = {0,0,0,0,0,0,0,0,0};
uint8_t position_covariance_type = 0;

using namespace std;

void getCmdOut(const char* cmd) {
    array<char, 128> buffer;
    string result;
    string temp;
    shared_ptr<FILE> pipe(popen(cmd, "r"), pclose);
    if (!pipe) throw std::runtime_error("popen() failed!");
    while (!feof(pipe.get())) {
        if (fgets(buffer.data(), 128, pipe.get()) != nullptr) {
            temp = buffer.data();
            if (temp.find("GPGGA") != std::string::npos) {
                result += temp;
                latitude = atof(temp.substr(14,13).c_str());
                longitude = atof(temp.substr(28,14).c_str());
                altitude = atof(temp.substr(49,5).c_str());
                navStatus.status = atoi(temp.substr(7,1).c_str());
                navStatus.service = 1;
            }
        }
    }
    //return result;
}

int main( int argc, char **argv)
{
    ros::init(argc, argv, "gpsNode");
    ros::NodeHandle n;
    ros::Publisher gpsPub = n.advertise<sensor_msgs::NavSatFix>("gpsmeas", 1000);
    ros::Rate loop_rate(1);

    int count = 0;
    thread t1(getCmdOut,"nc localhost 20175 ");
    while ( ros::ok())
    {
        sensor_msgs::NavSatFix msg;
        msg.position_covariance[0] = 0;
        msg.latitude = latitude;
        msg.longitude = longitude;
        msg.altitude = altitude;
        msg.position_covariance_type = 0;
        msg.status = navStatus;

 
         
        gpsPub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }
    t1.join();
    return 0;
}

