#include "mbed.h"
#include "tfmini.h"
#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle  nh;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

char hello[16] = "Hello from mBed";
char msg[10];
Serial connection(p13,p14);

DigitalOut led = LED1;
uint16_t dist;

int main() {
    nh.initNode();
    nh.advertise(chatter);
    
    connection.baud(TFMINI_BAUDRATE);
    TFMini tfm(&connection);

    while (1) {
        //usensor.start();
        //dist=usensor.get_dist_cm();
        dist = tfm.getDistance();
        
        sprintf(msg, "%d cm", dist);
  //      led = !led;
        str_msg.data = msg;
        chatter.publish( &str_msg );
        nh.spinOnce();
        wait_ms(500);
    }
}