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

using namespace std;


std::mutex gpsData; //Mutex for saving lat and long data

string latitude, longitude, altitude;
int status;
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
            if (temp.find("GPGGA") != std::string::npos) {
                result += temp;
                latitude = temp.substr(14,13);
                longitude = temp.substr(28,14);
                altitude = temp.substr(49,5);
                std::cout<<temp.substr(7,1)<<endl;
                status = atoi(temp.substr(7,1).c_str());
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
        std::cout << "status = " << status << endl;
        std::cin.ignore();
    }
    //std::cout << longitude << std::endl;
    t1.join();
    return 0;
}
