#include "ros/ros.h"
#include "can_msgs/Frame.h"
#include <sstream>
#include <test_can/LibCan.h>
#include <ros/console.h>

using namespace std;

ros::Publisher can_send;
ros::Publisher imu_pub;

float quaternion[4] = {0};

void can_callback(const can_msgs::Frame::ConstPtr& msg) 
{
    std::stringstream ss;
    if (msg->id == 0x34)   // MPU upload
    {
        for (int i = 0 ; i < 4 ; i ++ )
        {
            if(msg->data[2*i] == 0xff)
            {
                quaternion[i] = (float)(0xff - msg->data[2 * i + 1] + 1) / 0x100;
                quaternion[i] *= -1;
            }
            else quaternion[i] = (float)(msg->data[2 * i + 1])/ 0x100;
        }
      }
}


void imu_cmd()
{
    can_msgs::Frame msg;
    msg.id = 0x33;
    msg.data = {0x01,0,0,0,0,0,0,0};   //Get data from IMU
    msg.dlc = 8;
    can_send.publish(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "movementService");
    
    ros::NodeHandle n;
    LibCan libCan(n, n);
//  ros::Subscriber sub = n.subscribe("received_messages", 1000, can_callback);
//  can_send = n.advertise<can_msgs::Frame>("sent_messages", 1000);

    ros::Rate loop_rate(1);
    can_msgs::Frame msg;
    while(ros::ok())
    {
        ROS_INFO("RPM: %i %i %i. Vx: %f and Omega: %f",libCan.getRpm1(), libCan.getRpm2(), libCan.getRpm3(), libCan.getVelX(), libCan.getOmega());
        loop_rate.sleep();
        ros::spinOnce();
    }

  return 0;
}
