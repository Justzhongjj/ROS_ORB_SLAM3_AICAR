//
// Created by lsg on 2024/3/21.
//

#ifndef AICAR_CAN_H
#define AICAR_CAN_H

//样例只是提供一个简单的调用so库的方法供参考，程序接收，与发送函数设置在两个线程中，并且线程没有同步。
//现实中客户编程中，发送与接收函数不能同时调用（不支持多线程），如果在多线程中，一定需要互锁。需要客户自行完善代码。


#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>
#include "controlcan.h"
#include <iostream>
#include <ctime>
#include <cstdlib>
#include "unistd.h"
#include <vector>
using namespace std;

void *receive_func(void* param);  //接收线程。


bool Controlcan();

VCI_CAN_OBJ Create_Send(vector<int> data);

void Show_Transmit(VCI_CAN_OBJ send);

void CAN_Close();


////////Stop/////////////

static VCI_CAN_OBJ Stop = Create_Send({0,0,0,0,0,0,0,0});

///////////////////////


///////Forward/////////////////
static VCI_CAN_OBJ Forward_30 = Create_Send({0,30,0,0,0,0,0,0});
static VCI_CAN_OBJ Forward_40 = Create_Send({0,40,0,0,0,0,0,0});
static VCI_CAN_OBJ Forward_50 = Create_Send({0,50,0,0,0,0,0,0});
static VCI_CAN_OBJ Forward_60 = Create_Send({0,60,0,0,0,0,0,0});
static VCI_CAN_OBJ Forward_70 = Create_Send({0,70,0,0,0,0,0,0});
static VCI_CAN_OBJ Forward_100 = Create_Send({0,100,0,0,0,0,0,0});

///////Back/////////////////
// 速度有点快，
static VCI_CAN_OBJ Back_10 = Create_Send({-1,0,0,0,0,0,0,0});



///////Left/////////////////

static VCI_CAN_OBJ Left_10 = Create_Send({{0,0,0,0,10,0,0,0}});
static VCI_CAN_OBJ Left_20 = Create_Send({{0,0,0,0,20,0,0,0}});
static VCI_CAN_OBJ Left_30 = Create_Send({{0,30,0,0,30,0,0,0}});
static VCI_CAN_OBJ Left_40 = Create_Send({{0,0,0,0,40,0,0,0}});
static VCI_CAN_OBJ Left_50 = Create_Send({{0,0,0,0,50,0,0,0}});




///////Right/////////////////

static VCI_CAN_OBJ Right_10 = Create_Send({{0,0,0,0,-10,0,0,0}});
static VCI_CAN_OBJ Right_20 = Create_Send({{0,0,0,0,-20,0,0,0}});
static VCI_CAN_OBJ Right_30 = Create_Send({{0,30,0,0,-30,0,0,0}});
static VCI_CAN_OBJ Right_40 = Create_Send({{0,0,0,0,-40,0,0,0}});
static VCI_CAN_OBJ Right_50 = Create_Send({{0,0,0,0,-50,0,0,0}});



#endif //AICAR_CAN_H
