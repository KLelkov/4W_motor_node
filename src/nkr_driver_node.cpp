//#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <cstring>
#include <iostream>
//#include <rdk_msgs/control.h>
//#include <rdk_msgs/motors.h>
//#include <std_msgs/String.h>
#include "CppSerial.cpp"


class SubscribeAndPublish
{
private:
  CppSerial ser;
  unsigned char read_buffer[256] = {};
  float odoFrontLeft = 0.0;
  float odoFrontRight = 0.0;
  float odoRearLeft = 0.0;
  float odoRearRight = 0.0;
  float odoAngleFront = 0.0;
  float odoAngleRear = 0.0;

  int controlFrontLeft = 0;
  int controlFrontRight = 0;
  int controlRearLeft = 0;
  int controlRearRight = 0;
  int controlAngle = 0;
  int controlBlock = 1;

  bool calibrationNeeded = false;


public:
    SubscribeAndPublish() // This is the constructor
    {
      char portname[] = "/dev/ttyUSB0";
      ser.Open(portname, 115200);
      ser.Flush();
      //keyboardSub = n_.subscribe("keyboard_commands", 1, &SubscribeAndPublish::keyboard_callback, this);
      //controlSub = n_.subscribe("control_commands", 1, &SubscribeAndPublish::control_callback, this);
      //odoPub = n_.advertise<rdk_msgs::motos>("motors_data", 2);
    }

    /*void control_callback(const rdk_msgs::control msg)
    {

    }

    void keyboard_callback(const std_msgs::String msg)
    {

    }*/

    void sendControl()
    {
      char MSG[128] = {'\0'};
      memset(MSG, 0, sizeof(MSG));
      sprintf(MSG, "[drv] %d %d %d %d %d %d\n", controlFrontLeft, controlFrontRight, controlRearLeft, controlRearRight, controlAngle, controlBlock);
      // TODO: find a secure way to build (unsigned char) string instead
      // of converting a (char) one.
      ser.Send((unsigned char*)MSG, sizeof(MSG));
      std::cout << "Sent: " << (unsigned char*)MSG << std::endl;

      if (calibrationNeeded)
      {
        std::cout << "Axis calibration in process..." << std::endl;
        usleep(1000000);
        unsigned char calmsg[] = "[cal] \n";
        ser.Send(calmsg, sizeof(calmsg));
      }

    }

    void readOdometry()
    {
        int len = ser.Read(read_buffer);
        std::cout << read_buffer << std::endl;
        // Parse the data...

        // Publish odometry...

        //ser.Flush();
    }


};//End of class SubscribeAndPublish

int main(int argc, char** argv)
{
    //ros::init(argc, argv, "nkr_driver_node");

    // Handle ROS communication events
    SubscribeAndPublish SAPObject;

    // For cycling operation use
    //ros::Rate rate(20); // ROS Rate at 20 Hz
    //while (ros::ok()) {

    while (true)
    {
      // sendControl() should flush uart transmit before sending data
      //SAPObject.sendControl();
      SAPObject.readOdometry();

      // readOdometry() should flush uart receive upon completion
    //  rate.sleep()
      usleep(500000);
    }


  	return 0;
  }
