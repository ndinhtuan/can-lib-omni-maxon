#include <test_can/LibCan.h>
#include <iostream>
#include "ros/ros.h"
#include "can_msgs/Frame.h"
#include <time.h>
#include <cstring>

using namespace std;

LibCan::LibCan(ros::NodeHandle _pubNodeHandle, ros::NodeHandle _subNodeHandle) : pubNodeHandle(_pubNodeHandle), subNodeHandle(_subNodeHandle)
{
    this->pub = pubNodeHandle.advertise<can_msgs::Frame>("sent_messages", 1000);
    this->sub = subNodeHandle.subscribe("received_messages", 100, &LibCan::callbackReceivedMessagesCan, this);
}

LibCan::LibCan(){}

// 8 bytes data of can
int LibCan::_decodeCandata(const can_msgs::Frame::ConstPtr &msg) {

    // 4 first byte is byte for data, negative when dataCan[2] and dataCan[3] is 0xFF
    int data = ((msg->data[1] << 8) | (0xFF & msg->data[0]));
    if (msg->data[3]==0xFF){
        data=data-0xFFFF-1;
    }

    return data;
}

void LibCan::sendSyncMsg()
{
    can_msgs::Frame msg;
    msg.id = 0x080;
    msg.dlc = 0;
    msg.data = {0, 0, 0, 0, 0, 0, 0, 0};
    this->pub.publish(msg);
}

void LibCan::sendVelMsg(int vel, int motorId)
{
    int data0, data1, data2, data3;
    can_msgs::Frame msg;
    /* Set msg's receiving address */
    switch(motorId)
    {
        case 1:
        {
            msg.id = 0x221;
            break;
        }
        case 2:
        {
            msg.id = 0x222;
            break;
        }
        case 3:
        {
            msg.id = 0x223;
            break;
        }
    }
    /* Set msg's data length */
    msg.dlc = 8;
    /* Set msg's data */
    if(vel >= 0)
    {
        data0 = (vel & 0xFF);
        data1 = (vel >> 8);
        data2 = 0x00;
        data3 = 0x00;
    }
    else
    {
        int _vel = 0xFFFF - (-vel) + 1;
        data0 = (_vel & 0xFF);
        data1 = (_vel >> 8);
        data2 = 0xFF;
        data3 = 0xFF;
    }
    msg.data = {data0, data1, data2, data3, 0x0F, 0x00, 0x00, 0x00};
    /* Send msg */
    this->pub.publish(msg);
    usleep(10000);
    this->sendSyncMsg();
    usleep(10000);
}

void LibCan::initMotorDriver()
{
	can_msgs::Frame msg;
     /* All Enter Opn */
    msg.id = 0x000;
    msg.dlc = 2;
    msg.data = {0x01, 0x00};
    this->pub.publish(msg);
    usleep(100000);
    this->sendSyncMsg();
    usleep(10000);

    /* Fault Reset */
    msg.id = 0x601;
    msg.dlc = 8;
    msg.data = {0x2B, 0x40, 0x60, 0x00, 0x80, 0x00, 0x00, 0x00};
    this->pub.publish(msg);
    usleep(100000);
    this->sendSyncMsg();
    usleep(10000);
    
    msg.id = 0x602;
    msg.dlc = 8;
    msg.data = {0x2B, 0x40, 0x60, 0x00, 0x80, 0x00, 0x00, 0x00};
    this->pub.publish(msg);
    usleep(100000);
    this->sendSyncMsg();
    usleep(10000);

    msg.id = 0x603;
    msg.dlc = 8;
    msg.data = {0x2B, 0x40, 0x60, 0x00, 0x80, 0x00, 0x00, 0x00};
    this->pub.publish(msg);
    usleep(100000);
    this->sendSyncMsg();
    usleep(10000);

    /* EPOS Shut Down */
    msg.id = 0x601;
    msg.dlc = 8;
    msg.data = {0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00};
    this->pub.publish(msg);
    usleep(100000);
    this->sendSyncMsg();
    usleep(10000);

    msg.id = 0x602;
    msg.dlc = 8;
    msg.data = {0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00};
    this->pub.publish(msg);
    usleep(100000);
    this->sendSyncMsg();
    usleep(10000);

    msg.id = 0x603;
    msg.dlc = 8;
    msg.data = {0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00};
    this->pub.publish(msg);
    usleep(100000);
    this->sendSyncMsg();
    usleep(10000);

    /* Enable Operation */
    msg.id = 0x601;
    msg.dlc = 8;
    msg.data = {0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00};
    this->pub.publish(msg);
    usleep(100000);
    this->sendSyncMsg();
    usleep(10000);
    
    msg.id = 0x602;
    msg.dlc = 8;
    msg.data = {0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00};
    this->pub.publish(msg);
    usleep(100000);
    this->sendSyncMsg();
    usleep(10000);
    
    msg.id = 0x603;
    msg.dlc = 8;
    msg.data = {0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00};
    this->pub.publish(msg);
    usleep(100000);
    this->sendSyncMsg();
    usleep(10000);
}

void LibCan::callbackReceivedMessagesCan(const can_msgs::Frame::ConstPtr& msg){

    int data = this->_decodeCandata(msg);

    switch(msg->id) {
        case 0x1A1:
            this->rpm1 = data;
        break;
        case 0x1A2:
            this->rpm2 = data;
        break;
        case 0x1A3:
            this->rpm3 = data;
        break;
    }
}
// alpha1 = 60, alpha2=180, alpha3=300
// R=0.065m, L=0.26m
// Before computing, should convert rpm to rps (min to second)
float LibCan::getVelX(){
    // return 0.00664972*(this->rpm3-this->rpm1)/(2*27.277);
    return (0.065/(3*60))*(0.866*(this->rpm3) + -0.866*(this->rpm1));
}

float LibCan::getOmega(){
    // return 0.00664972*this->rpm2/3.1547;
    return -(0.065/(3*0.26))*(this->rpm1 + this->rpm2 + this->rpm3);
}

bool LibCan::checkRedLed()
{
}

int LibCan::getCurrentFeed()
{
}

int LibCan::getVoltageFeed()
{
}

