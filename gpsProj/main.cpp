/* Program to read input from Android GPS application SHARE GPS
 * Latest Author: Andrew Tzeng
 *
 * REQUIREMENTS:
 * SHARE GPS on Android Phone
 * Open Share GPS and click on connection to enable listening
 * Run gpsBash script
 */

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
#include <mutex>

#include "ros/ros.h"
#include "sensor_msgs

using namespace std;


std::mutex gpsData; //Mutex for saving lat and long data

float latitude, longitude;
int timestamp;

/* getCmdOut Method
 *
 * Input: Shell Command
 * Global variables timestamp, longitude, and latitude are edited in function
 *
 */

void getCmdOut(const char* cmd) {
    array<char, 128> buffer;
    string result;
    string temp;
    shared_ptr<FILE> pipe(popen(cmd, "r"), pclose);
    if (!pipe) throw std::runtime_error("popen() failed!");
    while (!feof(pipe.get())) {
        if (fgets(buffer.data(), 128, pipe.get()) != nullptr) {
            temp = buffer.data();
            if (temp.find("GPRMC") != std::string::npos) {
                result += temp;
                timestamp = atoi(temp.substr(7, 6).c_str());
                latitude = atof(temp.substr(16,11).c_str());
                longitude = atof(temp.substr(30,11).c_str());
            }
        }
    }
    //return result;
}

int main() {
    //string response =getCmdOut("nc localhost 20175 ");
    thread t1(getCmdOut,"nc localhost 20175 ");
    //std::cout << latitude << std::endl;
    while(1) {
        std::cout << "latitude = " << latitude << endl;
        std::cin.ignore();
    }
    //std::cout << longitude << std::endl;
    t1.join();
    return 0;
}