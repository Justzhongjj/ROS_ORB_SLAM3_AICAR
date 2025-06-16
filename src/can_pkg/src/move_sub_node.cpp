//
// Created by zjj on 2024/4/10.
// 在ROS的基础上增加键盘控制
// key_ctrl_pkg键盘发布话题
// can_pkg 订阅/keyboard_ctrl 控制小车


#include<iostream>
#include "../include/can_pkg/move.h"
#include"ros/ros.h"
#include "std_msgs/Int32.h"

#include<nav_msgs/Odometry.h>
#include<geometry_msgs/Twist.h>
#include <../include/can_pkg/can.h>
int a = 50;
int b = 0;
VCI_CAN_OBJ Forward_ = Create_Send({0,a,0,0,0,0,0,0});
using namespace std;
void carControl(const std_msgs::Int32 & msg){
    /*
        msg.data:
                1:"CLOSE"   //关闭
                2:"START"   //启动
                3:"STOP"    //停止
                4:"UP"      //前行
                5:"DOWN"    //后退
                6:"LEFT"    //左转
                7:"RIGHT"   //右转
               
    */
    switch (msg.data)
    {
        case 1:
                Move_Stop(Stop);
                CAN_Close();
                break;
        case 2:
                Controlcan();
                break;
        case 3:
                Move_Stop(Stop);
                break;
        case 4:
        	a = a + 10;
        	Forward_ = Create_Send({1,a,0,0,0,0,0,0});
                Forward(Forward_);
                Show_Transmit(Forward_);
                break;
        case 5:
                Back(Back_10);
                Show_Transmit(Back_10);
                break; 
        case 6:
                Left(Left_30);
                Show_Transmit(Left_30);
               // usleep(1000000);
                break;
        case 7:
                Right(Right_30);
                Show_Transmit(Right_30);
               // usleep(1000000);
                break;
        default:
                break;
    }
}


void cmdVelCallback(const geometry_msgs::Twist& cmd_vel){
        ROS_INFO("Received a /cmd_vel message!");
        ROS_INFO("Linear Components:[%f,%f,%f]",cmd_vel.linear.x,cmd_vel.linear.y,cmd_vel.linear.z);
        ROS_INFO("Angular Components:[%f,%f,%f]",cmd_vel.angular.x,cmd_vel.angular.y,cmd_vel.angular.z);
	
        if(cmd_vel.linear.x>0&&cmd_vel.angular.z==0){
                Forward(Forward_50);
        }
        else if(cmd_vel.linear.x>0&&cmd_vel.angular.z<0){
                Left(Left_30);
                Show_Transmit(Left_30);
                usleep(1000000);
        }
        else if(cmd_vel.linear.x>0&&cmd_vel.angular.z>0){
                Right(Right_30);
                Show_Transmit(Left_30);
                usleep(1000000);
        }
        else if(cmd_vel.linear.x==0){
                Move_Stop(Stop);
                usleep(1000000);
        }else{
                Back(Back_10);
                Show_Transmit(Back_10);
        }    

}


int main(int argc, char **argv)
{

  
  ros::init(argc, argv, "move_sub");
  ros::NodeHandle nh;
  ros::Subscriber move_sub_msg = nh.subscribe("/keyboard_ctrl", 100, carControl);
  //ros::Subscriber cmd_vel_sub = nh.subscribe("/cmd_vel", 5, cmdVelCallback);


  ros::spin();
  return 0;
}
