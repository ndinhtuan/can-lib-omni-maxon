#ifndef LIBCAN_H
#define LIBCAN_H

#include <iostream>
#include "ros/ros.h"
#include "can_msgs/Frame.h"

class LibCan
{
    ros::Publisher pub;
    ros::Subscriber sub;

private:
    ros::NodeHandle pubNodeHandle;
    ros::NodeHandle subNodeHandle;
    int rpm1, rpm2, rpm3;

    // parameter of omni wheel system, can see image to imagine
    int _decodeCandata(const can_msgs::Frame::ConstPtr& msg);
public:
    /* Constructor */
    LibCan(ros::NodeHandle _pubNodeHandle, ros::NodeHandle _subNodeHandle);
    LibCan();

    void sendSyncMsg();
    
    void sendVelMsg(int vel, int motorId);
    
    void initMotorDriver();

    void callbackReceivedMessagesCan(const can_msgs::Frame::ConstPtr& msg);

    int getRpm1(){ return this->rpm1;}
    int getRpm2(){ return this->rpm2;}
    int getRpm3(){ return this->rpm3;}

    float getVelX(); // Get velocity of x axis
    float getOmega(); // Get omega angle of robot
    /*
    Algorithm to check red led:     
    + Can check no message feed back to CAN when we try send some message
    @Return True if led is red, False if otherwise
    */
    bool checkRedLed();
    /*
    @Return current type int
    */
    int getCurrentFeed();
    /*
    @Return Voltage type int
    */
    int getVoltageFeed();        
};
#endif
