// Start of Gps Node Program
// Inputs: Gps data from Android phone running ShareGPS
// Outputs: GPS Message of type sensor_msgs/NavSatFix.msg

#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/NavSatStatus.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <std_msgs/Float64.h>
#include "std_msgs/Header.h"
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <cstdio>
#include <iostream>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <array>
#include <thread>
#include <bits/stdc++.h>
#include <vector>
#include <cmath>

using namespace std;

ros::Publisher wpPub;
int counter = 0;

vector<vector<double>> getwaypoints() {
    /*
    ifstream file("~/mARVin/catkin_ws/src/marvin_gps/src/gtriTest1.txt");
    cout << file.is_open() << "\n";
    string str;
    vector<string> fileContents;
    while (getline(file,str)) {
        cout << str << "\n";
        fileContents.push_back(str);
    }
    int numLines = fileContents.size();
    cout << "Number of lines: " << numLines << "\n";
    */
    vector<vector<double>> waypointPath(4, vector<double>(3,1));
    waypointPath[0][0] = 33.78055;
    waypointPath[0][1] = -84.40152;
    waypointPath[0][2] = 283.7;

    waypointPath[1][0] = 33.78055;
    waypointPath[1][1] = -84.40136;
    waypointPath[1][2] = 283.2;

    waypointPath[2][0] =33.78082; 
    waypointPath[2][1] =-84.40135;
    waypointPath[2][2] =283.1;

    waypointPath[3][0] =33.78083;
    waypointPath[3][1] =-84.40152;
    waypointPath[3][2] =283.9;
    return waypointPath;
    /*

    if ((numLines+1)%4 != 0) {
        cout << "incorrect file format. enter each gps waypoint (long,lat, and alt) as 3 separate lines followed by a blank line.\n";
        return waypointPath;
    } else {
        vector<double> waypoint;
        int count = 0;
        for (vector<string>::iterator it = fileContents.begin(); it != fileContents.end(); ++it) {
	    if (count < 3) {
                waypoint.push_back(stod(*it));
                count++;
            } else {
                count = 0;
                waypointPath.push_back(waypoint);
                waypoint.clear();
            }
            
        }
        return waypointPath;
    } 
    */      
    
}


void callback(const nav_msgs::Odometry &odom) {
    cout << "Waypoint received: (" << odom.pose.pose.position.x << ", " << odom.pose.pose.position.y << ")\n";
    geometry_msgs::PoseWithCovarianceStamped posCovStamped;
    posCovStamped.pose = odom.pose;
    posCovStamped.header = odom.header;
    wpPub.publish(posCovStamped);
    counter++;
}


int main( int argc, char **argv)
{
    ros::init(argc, argv, "set_waypoints");
    ros::NodeHandle n;
    ros::Publisher gpsPub = n.advertise<sensor_msgs::NavSatFix>("fix", 10);

    cout << "getting waypoints. \n";
    vector<vector<double>> waypointPath = getwaypoints();

    sensor_msgs::NavSatStatus navStatus;
    navStatus.service = 1;
    navStatus.status = 1;float latitude, longitude, altitude;
    float position_covariance[9] = {0,0,0,0,0,0,0,0,0};
    uint8_t position_covariance_type = 0;
    std_msgs::Header header;
    float hdop = 1.5;
    wpPub = n.advertise<nav_msgs::Odometry>("initialpose", 10);
    cout << "starting subscriber to /gps \n";
    ros::Subscriber odomSub = n.subscribe("gps", 10, callback); 
    ros::Rate r(10);
    while(1) {
    for (vector<vector<double>>::iterator it1 = waypointPath.begin(); it1 != waypointPath.end(); ++it1) {
	longitude = (*it1)[0];
	latitude = (*it1)[1];
	altitude = (*it1)[2];
	header.stamp = ros::Time::now();
	header.frame_id = 1;
	sensor_msgs::NavSatFix msg;
	msg.position_covariance[0] = pow(hdop,2);
	msg.position_covariance[4] = pow(hdop,2);
	msg.position_covariance[8] = pow((2*hdop),2);
	msg.latitude = latitude;
	msg.longitude = longitude;
	msg.altitude = altitude;
	msg.position_covariance_type = 1;
	msg.status = navStatus;
	msg.header = header;
	gpsPub.publish(msg);
        r.sleep();
	
    } 
    }

    
    return 0;
}

