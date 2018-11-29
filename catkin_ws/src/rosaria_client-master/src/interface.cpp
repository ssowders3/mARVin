#include <ros/ros.h>
#include <iostream>
#include <stdlib.h>
#include <termios.h>
#include <string>
#include <unistd.h>
#include <signal.h>
#include <rosaria_client/waypoint.h>
#define KEYCODE_1 0x31
#define KEYCODE_2 0X32
#define KEYCODE_3 0x33
#define KEYCODE_4 0x34
#define KEYCODE_5 0x35
#define KEYCODE_6 0x36
#define KEYCODE_7 0x37
#define KEYCODE_Q 0X71

static struct termios new1, old;

/* initializes the terminal to new input settings */
void initTermios(int echo)
{
    tcgetattr(0, &old); /* grab old terminal i/o settings */
    new1 = old; /* make new settings same as old settings */
    new1.c_lflag &= ~ICANON; /* disable buffered i/o */
    new1.c_lflag &= echo ? ECHO : ~ECHO; /* set echo mode */
    tcsetattr(0, TCSANOW, &new1); /* use these new terminal i/o settings now */
}

/* Restore old terminal i/o settings */
void resetTermios()
{
    tcsetattr(0, TCSANOW, &old);
}

/* quit the program cleanly and close ros */
void quit()
{
	resetTermios(); /* reset the terminal to old settings */
	ros::shutdown(); /*shutdown ros */
	exit(1); /* exit the system */
}

/* Handles top level control as usual */
int main(int argc, char **argv)
{
  /* initialize the ros node */
	ros::init(argc, argv, "RosAria_interface");
	ros::NodeHandle nh;

  ros::Publisher waypoint_publish = nh.advertise<rosaria_client::waypoint>("/waypointMsg",1);

  /* change the terminal input settings */
	initTermios(0);

  /* greet user and display selection options */
	std::cout
			<< "******************************************************************" << std::endl
			<< "*                   ROSARIA CLIENT INTERFACE                     *" << std::endl
			<< "*                                                                *" << std::endl
			<< "*            Welcome to the RosAria client interface!            *" << std::endl
			<< "*                                                                *" << std::endl
			<< "*       [1] go_three_second                                      *" << std::endl
			<< "*       [2] spin_clockwise                                       *" << std::endl
			<< "*       [3] spin_counterclockwise                                *" << std::endl
			<< "*       [4] teleop                                               *" << std::endl
			<< "*       [5] enable/disable print_state                           *" << std::endl
			<< "*       [6] enable_motors                                        *" << std::endl
      << "*       [7] enter waypoints                                      *" << std::endl
			<< "*       Press [Q] to close the interface                         *" << std::endl
			<< "******************************************************************" << std::endl;

	char select,a,b; /* vars to be used in switch statement */
  std::string waypoint_num, temp_latitude, temp_longitude, temp_altitude;
  typedef struct {
    float latitude;
    float longitude;
    float altitude;
  } waypoints_t;
  waypoints_t waypoints[100];
  rosaria_client::waypoint msg;

  /* loop to handle user input */
	while(ros::ok()){
		std::cout << "Please select a program to run, or hit q to quit: "<< std::endl; /* prompt user at start of every loop */
		std::cin >> select;	/* use standard input to select program to run */
		switch(select)
		{
			case KEYCODE_1:
			{
				pid_t pid; /*gets the pid */
				switch(pid = fork()) /* forks the process */
				{
					case 0: /* when pid == 0, we have the child process */
						b = system("rosrun rosaria_client print_state"); /*run the print_state function */
						exit(1); /* exit the child process */
						break;
				}
				a = system("rosrun rosaria_client go_three_second"); /* run option 1*/
				a = system("rosnode kill /print_aria_state "); /*kill the ros print_state node */
			}
				break;
			case KEYCODE_2:
			{
				pid_t pid; /*gets the pid */
				switch(pid = fork()) /* forks the process */
				{
					case 0: /* when pid == 0, we have the child process */
						b = system("rosrun rosaria_client print_state"); /*run the print_state function */
						exit(1); /* exit the child process */
						break;
				}
				a = system("rosrun rosaria_client spin_clockwise"); /* run option 2 */
				a = system("rosnode kill /print_aria_state "); /*kill the ros print_state node */
				break;
			}
			case KEYCODE_3:
			{
				pid_t pid; /*gets the pid */
				switch(pid = fork()) /* forks the process */
				{
					case 0: /* when pid == 0, we have the child process */
						b = system("rosrun rosaria_client print_state"); /*run the print_state function */
						exit(1); /* exit the child process */
						break;
				}
				a = system("rosrun rosaria_client spin_counterclockwise"); /* run option 3 */
				a = system("rosnode kill /print_aria_state "); /*kill the ros print_state node */
				break;
			}
			case KEYCODE_4:
			{
				pid_t pid; /*gets the pid */
				switch(pid = fork()) /* forks the process */
				{
					case 0: /* when pid == 0, we have the child process */
						b = system("rosrun rosaria_client print_state"); /*run the print_state function */
						exit(1); /* exit the child process */
						break;
				}
				a = system("rosrun rosaria_client teleop"); /* run option 4 */
				a = system("rosnode kill /print_aria_state "); /*kill the ros print_state node */
				break;
			}
			case KEYCODE_5:

				break;
			case KEYCODE_6:
				a = system("rosrun rosaria_client enable_motors"); /* run option 6 */
				break;
      case KEYCODE_7:
        std::cout << "Please enter number of waypoints: " << std::endl;
        std::cin >> waypoint_num;
        if (std::atoi(waypoint_num.c_str()) > 100) {
          std::cout << "Error: cannot have more than 100 waypoints" << std::endl;
          break;
        }
        for (int i = 0; i < std::atoi(waypoint_num.c_str()); i++) {
          msg.waypoint_id = i;
          std::cout << "Enter Longitude: " << std::endl;
          std::cin >> temp_longitude;
          waypoints[i].longitude = std::atof(temp_longitude.c_str());
          msg.longitude = waypoints[i].longitude;
          std::cout << "Entered longitude: " << waypoints[i].longitude << std::endl;
          std::cout << "Enter Latitude: " << std::endl;
          std::cin >> temp_latitude;
          waypoints[i].latitude = std::atof(temp_latitude.c_str());
          msg.latitude = waypoints[i].latitude;
          std::cout << "Entered latitude: " << waypoints[i].latitude << std::endl;
          std::cout << "Enter Altitude: " << std::endl;
          std::cin >> temp_altitude;
          waypoints[i].altitude = std::atof(temp_altitude.c_str());
          msg.altitude = waypoints[i].altitude;
          std::cout << "Entered Altitude: " << waypoints[i].altitude << std::endl;
          waypoint_publish.publish(msg);
        }

        break;
			case KEYCODE_Q: /* quit the interface program */
        quit(); /* reset the terminal, then quit + exit the program */
        return false;
				break;
			default: /* in case of user not entering selection from list */
				std::cout << "Please enter a number from the list or Q to quit." << std::endl;
				break;
		}
	}
	return 1;
}
