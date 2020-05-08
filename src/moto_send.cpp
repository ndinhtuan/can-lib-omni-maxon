#include "ros/ros.h"
#include <can_msgs/Frame.h>
#include <test_can/LibCan.h>
#include <iostream>
#include <cstring>
// #include <RobotControl.h>
#include <geometry_msgs/Twist.h>

using namespace std;
int vel_x = 0, ang_z = 0,vel_y = 0;
LibCan* CanManager;
void velCallback(const geometry_msgs::Twist& msg){
	//std::cout<<msg.linear.x<<"\t"<<msg.angular.z<<"\n";
	vel_x = (int) (msg.linear.x * 100);
	vel_y = (int) (msg.linear.y * 100);
	ang_z = (int) (msg.angular.z * 100);
//	std::cout<<vel_x<<"\t"<<ang_z<<"\n";
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "receive_node");
	ros::NodeHandle nodeHandle;
	ros::Subscriber vel_sub = nodeHandle.subscribe("cmd_vel", 0, velCallback);

    ros::NodeHandle pubNodeHandle;
    ros::NodeHandle subNodeHandle;
    ros::Rate loop_rate(100);

    CanManager = new LibCan(pubNodeHandle, subNodeHandle);
    CanManager->initMotorDriver();
    CanManager->initMotorDriver();
    int speed1=0, speed2=0, speed3=0;
    RPomniDirect rb(1.0f,1.0f,1.0f,50.0f); // tạo 1 đối tượng là rb có thuộc tính của class RPomniDirect
    while(ros::ok())
    {
		rb.move(vel_y, vel_x, ang_z, &speed2, &speed3, &speed1); 
		cout<<speed1<<"\t"<<speed2<<"\t"<<speed3<<"\n";
		//  ROS_INFO("ewqqwe");
		CanManager->sendVelMsg(speed1, 1);
		CanManager->sendVelMsg(speed2, 2);
		CanManager->sendVelMsg(speed3, 3);

		ros::spinOnce();
		loop_rate.sleep();
    }

    return 0;
}

