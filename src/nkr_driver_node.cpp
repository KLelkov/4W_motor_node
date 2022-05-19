#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <cstring>
#include <iostream>
#include <rdk_msgs/control.h>
#include <rdk_msgs/motors.h>
#include <std_msgs/String.h>
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
  int controlStop = 0;

  int a_controlFrontLeft = 0;
  int a_controlFrontRight = 0;
  int a_controlRearLeft = 0;
  int a_controlRearRight = 0;
  int a_controlAngle = 0;

  bool calibrationNeeded = false;

  ros::NodeHandle nh;
  ros::Subscriber keyboardSub;
  ros::Subscriber controlSub;
  ros::Publisher odoPub;


public:
    SubscribeAndPublish() // This is the constructor
    {
      char portname[] = "/dev/ttyUSB0";
      ser.Open(portname, 115200);

      odoPub = nh.advertise<rdk_msgs::motors>("motors_data", 2);
      keyboardSub = nh.subscribe<std_msgs::String>("keyboard_commands", 5, &SubscribeAndPublish::keyboard_callback, this);
      controlSub = nh.subscribe("control_commands", 5, &SubscribeAndPublish::control_callback, this);

    }

    int rps2duty(float radpersec)
    {
      int signum = (radpersec > 0) - (radpersec < 0);
      float duty_cycle = signum * (-0.000216 * pow(fabs(radpersec), 3) + 0.01554 * pow(radpersec, 2) + 1.8424 * fabs(radpersec) + 4.0242);
      if (fabs (duty_cycle) < 8)
      {
        duty_cycle = 0;
      }
      else if (fabs(duty_cycle) > 80)
      {
        signum = (duty_cycle > 0) - (duty_cycle < 0);
        duty_cycle = 80 * signum;
      }
      return (int)duty_cycle;
    }

    void control_callback(const rdk_msgs::control msg)
    {
      printf("\ncontrol callback\n");
      float time_diff = ros::Time::now().toSec() / 1000.0 - msg.timestamp;
      //if (fabs(time_diff) < 1000)
      if (true)
      {
        float rw = 0.254 / 2.0;
        float lf = 0.4;  // forward len
        float lr = 0.45;  // rear len
        float lw = 0.625;  // wheel base (full distsnce between wheels)
        // velocity2commands(data.ups, data.dth)
        if (msg.ups == 0) // cant turn on place (without moving forward)
        {
          if (msg.dth != 0)  // Fastest turn
          {
            int signum = (msg.dth > 0) - (msg.dth < 0);
            a_controlFrontLeft = 11;
            a_controlFrontRight = 11;
            a_controlRearLeft = 11;
            a_controlRearRight = 11;
            a_controlAngle = 25 * signum;
            return;
          }
          else  // Full stop
          {
            a_controlFrontLeft = 0;
            a_controlFrontRight = 0;
            a_controlRearLeft = 0;
            a_controlRearRight = 0;
            a_controlAngle = 0;
            return;
          }
        }
        else
        {
          float tan_gamma = msg.dth * (lf + lr) / fabs(msg.ups) / 2.0;
          a_controlAngle = atan(tan_gamma) * 180.0 / M_PI;
          if (a_controlAngle > 25)
          {
            a_controlAngle = 25 * ((msg.dth > 0) - (msg.dth < 0));
          }
          if (msg.dth == 0)
          {
            a_controlFrontLeft = rps2duty(msg.ups / rw);
            a_controlFrontRight = rps2duty(msg.ups / rw);
            a_controlRearLeft = rps2duty(msg.ups / rw);
            a_controlRearRight = rps2duty(msg.ups / rw);
          }
          else
          {
            float Rc = fabs(msg.ups / msg.dth);
            float signv = (msg.ups > 0) - (msg.ups < 0);
            float signr = (msg.dth > 0) - (msg.dth < 0);

            float vel1 = fabs(msg.dth) * sqrt( lf*lf + pow(Rc + lw / 2 * signr, 2) ) * signv;
            float vel2 = fabs(msg.dth) * sqrt( lf*lf + pow(Rc - lw / 2 * signr, 2) ) * signv;
            float vel3 = fabs(msg.dth) * sqrt( lr*lr + pow(Rc + lw / 2 * signr, 2) ) * signv;
            float vel4 = fabs(msg.dth) * sqrt( lr*lr + pow(Rc - lw / 2 * signr, 2) ) * signv;

            a_controlFrontLeft = rps2duty(vel1 / rw);
            a_controlFrontRight = rps2duty(vel2 / rw);
            a_controlRearLeft = rps2duty(vel3 / rw);
            a_controlRearRight = rps2duty(vel4 / rw);
          }
        }
      }
      return;
    }

    void keyboard_callback(const std_msgs::String msg)
    {
      //printf("keyboard %s", msg.data);
      if (msg.data.compare("block") == 0)
	    {
        controlBlock = abs(controlBlock - 1);
        if (controlBlock == 1)
          printf("\n[nkr_driver] Manual control mode is enabled.\n");
        else
          printf("\n[nkr_driver] Automatic control mode!\n");
      }
      else if (msg.data.compare("f") == 0)
      {
        controlFrontLeft += 5;
        controlFrontRight += 5;
        controlRearLeft += 5;
        controlRearRight += 5;
        controlStop = 0;
        if (controlFrontLeft > 80)
        {
          controlFrontLeft = 80;
          controlFrontRight = 80;
          controlRearLeft  = 80;
          controlRearRight = 80;
        }
      }
      else if (msg.data.compare("b") == 0)
      {
        controlFrontLeft -= 5;
        controlFrontRight -= 5;
        controlRearLeft -= 5;
        controlRearRight -= 5;
        controlStop = 0;
        if (controlFrontLeft < -80)
        {
          controlFrontLeft = -80;
          controlFrontRight = -80;
          controlRearLeft  = -80;
          controlRearRight = -80;
        }
      }
      else if (msg.data.compare("l") == 0)
      {
        controlAngle -= 5;
        if (controlAngle  < -30)
        {
          controlAngle = -30;
        }
      }
      else if (msg.data.compare("r") == 0)
      {
        controlAngle += 5;
        if (controlAngle  > 30)
        {
          controlAngle = 30;
        }
      }
      else if (msg.data.compare("s") == 0)
      {
        controlFrontLeft = 0;
        controlFrontRight = 0;
        controlRearLeft = 0;
        controlRearRight = 0;
        controlStop = 1;
      }
      else if (msg.data.compare("o") == 0)
      {
        controlFrontLeft = 0;
        controlFrontRight = 0;
        controlRearLeft = 0;
        controlRearRight = 0;
        controlStop = 1;
        controlAngle = 0;
        calibrationNeeded = true;
      }
    }

    void sendControl()
    {
      char MSG[128] = {'\0'};
      memset(MSG, 0, sizeof(MSG));
      if (controlBlock == 1)
      {
        sprintf(MSG, "[drv] %d %d %d %d %d %d\n", controlFrontLeft, controlFrontRight, controlRearLeft, controlRearRight, controlAngle, controlStop);
      }
      else
      {
        sprintf(MSG, "[drv] %d %d %d %d %d %d\n", a_controlFrontLeft, a_controlFrontRight, a_controlRearLeft, a_controlRearRight, a_controlAngle, 0);
      }

      // TODO: find a secure way to build (unsigned char) string instead
      // of converting a (char) one.
      ser.FlushTransmit();
      ser.Send((unsigned char*)MSG, sizeof(MSG));
      //ser.SendSigned(MSG, sizeof(MSG));
      std::cout << "Sent: " << MSG << std::endl;

      if (calibrationNeeded)
      {
        std::cout << "Axis calibration in process..." << std::endl;
        usleep(1000000);
        unsigned char calmsg[] = "[cal] \n";
        ser.Send(calmsg, sizeof(calmsg));
        calibrationNeeded = false;
      }
    }

    void readOdometry()
    {
        memset(read_buffer, 0, sizeof(read_buffer));
        int len = ser.ReadLine(read_buffer);
        std::cout << "Received: " << read_buffer << std::endl;
        // Parse the data...
        char cmd_buf[100] = {'\0'};
        strcpy(cmd_buf, (char *) read_buffer);
        char MSG[5] = {'\0'};
        //char msg;
	      float odo1 = 0, odo2 = 0, odo3 = 0, odo4 = 0;
	      float angf = 0, angr = 0;
	      sscanf(cmd_buf, "%s %f %f %f %f %f %f", MSG, &odo1, &odo2, &odo3, &odo4, &angf, &angr);
        if (!strcmp(MSG, "[enc]")) // returns 0 if strings are equal
        {
          rdk_msgs::motors msg;
          msg.timestamp = ros::Time::now().toSec() / 1000.0;
          msg.odo[0] = odo1;
          msg.odo[1] = odo2;
          msg.odoRear[0] = odo3;
          msg.odoRear[1] = odo4;
          msg.angleFront = angf;
          msg.angleRear = angr;
          // publish to /motors_data topic
          odoPub.publish(msg);
        }

        ser.FlushReceive();
    }


};//End of class SubscribeAndPublish

int main(int argc, char** argv)
{
    ros::init(argc, argv, "nkr_driver_node");

    // Handle ROS communication events
    SubscribeAndPublish SAPObject;

    // For cycling operation use
    ros::Rate rate(1); // ROS Rate at 20 Hz
    //while (ros::ok())
    while(!ros::isShuttingDown())
    {
      printf("deb: %d\n", ros::ok());
      // sendControl() should flush uart transmit before sending data
      SAPObject.sendControl();
      SAPObject.readOdometry();

      // readOdometry() should flush uart receive upon completion

     ros::spinOnce();
     rate.sleep();
    }

  	return 0;
  }
