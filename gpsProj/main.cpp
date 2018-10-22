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

string latitude, longitude;
int timestamp;

void getCmdOut(const char* cmd) {
    array<char, 128> buffer;
    string result;
    string temp;
    shared_ptr<FILE> pipe(popen(cmd, "r"), pclose);
    if (!pipe) throw std::runtime_error("popen() failed!");
    while (!feof(pipe.get())) {
        if (fgets(buffer.data(), 128, pipe.get()) != nullptr) {
            temp = buffer.data();
            if(temp.find("GPRMC") != std::string::npos){
                result += temp;
                timestamp = atoi(temp.substr(7,7).erase(6,7).c_str());
                latitude = temp.substr(16,16).erase();
                //longitude =
                //cout << latitude<<endl;
                //cout<<buffer.data()<<endl;
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
        std::cout << "timestamp = " << timestamp << endl;
        std::cin.ignore();
    }
    //std::cout << longitude << std::endl;
    t1.join();
    return 0;
}