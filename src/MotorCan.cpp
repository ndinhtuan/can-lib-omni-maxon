#include "ros/ros.h"
#include "can_msgs/Frame.h"
#include <iostream>
#include <time.h>
#include <geometry_msgs/Twist.h>
//#include "LibCan.h"

using namespace std;

void sendSyncMsg();
void sendVelMotor(int id, int rpm);
void resetMotor();

ros::Publisher sendCanMsg;
//ros::NodeHandle pubNodeHandle;
//ros::NodeHandle subNodeHandle;
//LibCan* CanManager = new LibCan(subNodeHandle, pubNodeHandle);

void velCallback(const geometry_msgs::Twist& msg){
	//ROS_INFO_STREAM("Subscriber velocities:"<<" linear="<<msg.linear.x<<" angular="<<msg.angular.z);
//	int w_l = ((msg.linear.x / 0.06) + (0.5 * msg.angular.z/ (2 * 0.06))) * 1000;
//	int w_r = -((msg.linear.x / 0.06) - (0.5 * msg.angular.z/ (2 * 0.06))) * 1000;
	float w1 = (27.272 * msg.linear.x - 3.1547 * msg.angular.z)/0.00664972;
	float w2 = -3.1397 * msg.angular.z / 0.00664972;
	float w3 = (-27.277 * msg.linear.x - 3.15468 * msg.angular.z)/0.00664972;
	
	int w1_rpm = (int) w1;
	int w2_rpm = (int) w2;
	int w3_rpm = (int) w3;
	//std::cout<<"w_1 = "<<w1_rpm<<"\tw_2 = "<<w2_rpm<<"\tw_3 = "<<w3_rpm<<"\n";	
	
	sendVelMotor(1, w1_rpm);
	sendVelMotor(2, w2_rpm);
	sendVelMotor(3, w3_rpm);
	
//	CanManager->sendVelMsg(w1_rpm, 1);
//	CanManager->sendVelMsg(w2_rpm, 2);
//	CanManager->sendVelMsg(w3_rpm, 3);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "motor_service");
    ros::NodeHandle nodeHandle;
    ros::Rate loop_rate(10);


    sendCanMsg = nodeHandle.advertise<can_msgs::Frame>("sent_messages", 1000);
    ros::Subscriber vel_sub = nodeHandle.subscribe("cmd_vel", 0, velCallback);
    resetMotor();
    resetMotor();
	std::cout<<"ewqeqw";
   // CanManager->initMotorDriver();
  //  CanManager->initMotorDriver();
    ros::spin();
}

void sendSyncMsg()
{
    can_msgs::Frame msg;
    msg.id = 0x080;
    msg.dlc = 0;
    msg.data = {0, 0, 0, 0, 0, 0, 0, 0};
    sendCanMsg.publish(msg);
}

void sendVelMotor(int id, int rpm)
{
    can_msgs::Frame msg;
    int x;
    if (rpm > 0 || rpm == 0)
	x = 0x00;
    else{
	x = 0xFF;
	rpm = 0xFFFF + rpm + 1;
	}
//    int a = rpm &0xff;
//    int b = rpm >> 8;
//    msg.data = {rpm & 0xff, rpm >> 8, x, x, 0x0F, 0x00, 0x00, 0x00};
//    std::cout<<hex<<a<<b<<x<<"\n";
    
    switch (id)
    {
        case 1:
        {   	
            msg.id = 0x221;
            msg.dlc = 8;
            msg.data = {rpm & 0xff, rpm >> 8, x, x, 0x0F, 0x00, 0x00, 0x00};
            break;
        }
        case 2:
        {
            msg.id = 0x222;
            msg.dlc = 8;
            msg.data = {rpm & 0xff, rpm >> 8, x, x, 0x0F, 0x00, 0x00, 0x00};
            break;
        }
        case 3:
        {
            msg.id = 0x223;
            msg.dlc = 8;
            msg.data = {rpm & 0xff, rpm >> 8, x, x, 0x0F, 0x00, 0x00, 0x00};
            break;
        }
		default: return;
    }
	sendCanMsg.publish(msg);
	usleep(10000);
        sendSyncMsg();
        usleep(10000);
}

void resetMotor() {
	// All Enter Opn
	usleep(100000);
	can_msgs::Frame msg;
	msg.id = 0x000;
	msg.dlc = 2;
	msg.data = {0x01, 0x00};
	sendCanMsg.publish(msg);

	// Fault Reset 
	usleep(100000);
	sendSyncMsg();
        usleep(10000);

	msg.id = 0x601;
	msg.dlc = 8;
	msg.data = {0x2B, 0x40, 0x60, 0x00, 0x80, 0x00, 0x00, 0x00};
	sendCanMsg.publish(msg);
	usleep(100000);
	sendSyncMsg();
        usleep(10000);

	msg.id = 0x602;
	msg.dlc = 8;
	msg.data = {0x2B, 0x40, 0x60, 0x00, 0x80, 0x00, 0x00, 0x00};
	sendCanMsg.publish(msg);
	usleep(100000);
	sendSyncMsg();
        usleep(10000);

	msg.id = 0x603;
	msg.dlc = 8;
	msg.data = {0x2B, 0x40, 0x60, 0x00, 0x80, 0x00, 0x00, 0x00};
	sendCanMsg.publish(msg);
	usleep(100000);
	sendSyncMsg();
        usleep(10000);
	//ShutDown

	msg.id = 0x601;
	msg.dlc = 8;
	msg.data = {0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00};
	sendCanMsg.publish(msg);
	usleep(100000);
	sendSyncMsg();
        usleep(10000);

	msg.id = 0x602;
	msg.dlc = 8;
	msg.data = {0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00};
	sendCanMsg.publish(msg);
	usleep(100000);
	sendSyncMsg();
        usleep(10000);

	msg.id = 0x603;
	msg.dlc = 8;
	msg.data = {0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00};
	sendCanMsg.publish(msg);
	usleep(100000);
	sendSyncMsg();
        usleep(10000);
	//Enable Operation

	msg.id = 0x601;
	msg.dlc = 8;
	msg.data = {0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00};
	sendCanMsg.publish(msg);
	usleep(100000);
	sendSyncMsg();
	usleep(10000);

	msg.id = 0x602;
	msg.dlc = 8;
	msg.data = {0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00};
	sendCanMsg.publish(msg);
	usleep(100000);
	sendSyncMsg();
	usleep(10000);

	msg.id = 0x603;
	msg.dlc = 8;
	msg.data = {0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00};
	sendCanMsg.publish(msg);
	usleep(100000);
	sendSyncMsg();
}

