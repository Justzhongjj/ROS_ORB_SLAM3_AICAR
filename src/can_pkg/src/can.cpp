//样例只是提供一个简单的调用so库的方法供参考，程序接收，与发送函数设置在两个线程中，并且线程没有同步。
//现实中客户编程中，发送与接收函数不能同时调用（不支持多线程），如果在多线程中，一定需要互锁。需要客户自行完善代码。

#include "../include/can_pkg/can.h"
#include <math.h>
VCI_BOARD_INFO pInfo;//用来获取设备信息。
int Send_count=0;//数据列表中，用来存储列表序号。
VCI_BOARD_INFO pInfo1 [50];
int num=0;

void *receive_func(void* param)  //接收线程。
{
    int reclen=0;
    VCI_CAN_OBJ rec[3000];//接收缓存，设为3000为佳。
    int i,j;

    int *run=(int*)param;//线程启动，退出控制。
    int ind=0;

    while((*run)&0x0f)
    {
        if((reclen=VCI_Receive(VCI_USBCAN2,0,ind,rec,3000,100))>0)//调用接收函数，如果有数据，进行数据处理显示。
        {
            for(j=0;j<reclen;j++)
            {
                printf("Index:%04d  ",Send_count);Send_count++;//序号递增
                printf("CAN%d RX ID:0x%08X", ind+1, rec[j].ID);//ID
                if(rec[j].ExternFlag==0) printf(" Standard ");//帧格式：标准帧
                if(rec[j].ExternFlag==1) printf(" Extend   ");//帧格式：扩展帧
                if(rec[j].RemoteFlag==0) printf(" Data   ");//帧类型：数据帧
                if(rec[j].RemoteFlag==1) printf(" Remote ");//帧类型：远程帧
                printf("DLC:0x%02X",rec[j].DataLen);//帧长度
                printf(" data:0x");	//数据
                for(i = 0; i < rec[j].DataLen; i++)
                {
                    printf(" %02X", rec[j].Data[i]);
                }
                printf(" TimeStamp:0x%08X",rec[j].TimeStamp);//时间标识。
                printf("\n");
            }
        }
        ind=!ind;//变换通道号，以便下次读取另一通道，交替读取。
    }
    printf("run thread exit\n");//退出接收线程
    pthread_exit(0);
}

bool Controlcan(){
    printf(">>this is hello !\r\n");//指示程序已运行

    num=VCI_FindUsbDevice2(pInfo1);

    printf(">>USBCAN DEVICE NUM:");printf("%d", num);printf(" PCS");printf("\n");

    if(VCI_OpenDevice(VCI_USBCAN2,0,0)==1)//打开设备
    {
        printf(">>open deivce success!\n");//打开设备成功
    }else
    {
        printf(">>open deivce error!\n");
        //exit(1);
        return -1;
    }
    if(VCI_ReadBoardInfo(VCI_USBCAN2,0,&pInfo)==1)//读取设备序列号、版本等信息。
    {
        printf(">>Get VCI_ReadBoardInfo success!\n");
        printf(">>Serial_Num:%c", pInfo.str_Serial_Num[0]);
        printf("%c", pInfo.str_Serial_Num[1]);
        printf("%c", pInfo.str_Serial_Num[2]);
        printf("%c", pInfo.str_Serial_Num[3]);
        printf("%c", pInfo.str_Serial_Num[4]);
        printf("%c", pInfo.str_Serial_Num[5]);
        printf("%c", pInfo.str_Serial_Num[6]);
        printf("%c", pInfo.str_Serial_Num[7]);
        printf("%c", pInfo.str_Serial_Num[8]);
        printf("%c", pInfo.str_Serial_Num[9]);
        printf("%c", pInfo.str_Serial_Num[10]);
        printf("%c", pInfo.str_Serial_Num[11]);
        printf("%c", pInfo.str_Serial_Num[12]);
        printf("%c", pInfo.str_Serial_Num[13]);
        printf("%c", pInfo.str_Serial_Num[14]);
        printf("%c", pInfo.str_Serial_Num[15]);
        printf("%c", pInfo.str_Serial_Num[16]);
        printf("%c", pInfo.str_Serial_Num[17]);
        printf("%c", pInfo.str_Serial_Num[18]);
        printf("%c", pInfo.str_Serial_Num[19]);printf("\n");

        printf(">>hw_Type:%c", pInfo.str_hw_Type[0]);
        printf("%c", pInfo.str_hw_Type[1]);
        printf("%c", pInfo.str_hw_Type[2]);
        printf("%c", pInfo.str_hw_Type[3]);
        printf("%c", pInfo.str_hw_Type[4]);
        printf("%c", pInfo.str_hw_Type[5]);
        printf("%c", pInfo.str_hw_Type[6]);
        printf("%c", pInfo.str_hw_Type[7]);
        printf("%c", pInfo.str_hw_Type[8]);
        printf("%c", pInfo.str_hw_Type[9]);printf("\n");

        printf(">>Firmware Version:V");
        printf("%x", (pInfo.fw_Version&0xF00)>>8);
        printf(".");
        printf("%x", (pInfo.fw_Version&0xF0)>>4);
        printf("%x", pInfo.fw_Version&0xF);
        printf("\n");
    }else
    {
        printf(">>Get VCI_ReadBoardInfo error!\n");
        return -1;
    }

    //初始化参数，严格参数二次开发函数库说明书。
    VCI_INIT_CONFIG config;
    config.AccCode=0;
    config.AccMask=0xFFFFFFFF;
    config.Filter=1;//接收所有帧
    config.Timing0=0x00;/*波特率1000 Kbps  0x00  0x14*/
    config.Timing1=0x14;
    config.Mode=0;//正常模式


    if(VCI_InitCAN(VCI_USBCAN2,0,1,&config)==1)
    {
        printf("调用 VCI_InitCAN2 成功\n");
    }
    else{
        cout << "调用 VCI_InitCAN2 失败\n" << endl;
        return -1;
    }
    if(VCI_StartCAN(VCI_USBCAN2,0,1)==1)
    {
        printf("调用 VCI_StartCAN2 成功\n");
    }
    else{
        cout << "调用 VCI_StartCAN2 失败\n" << endl;
        return -1;
    }
    return 1;
}

//创建控制消息
VCI_CAN_OBJ Create_Send(vector<int> data){
    VCI_CAN_OBJ send[1];
    send[0].ID=0X0181;
    send[0].SendType=0;
    send[0].RemoteFlag=0;
    send[0].ExternFlag=0;
    send[0].DataLen=8;


    for(int i = 0; i < send[0].DataLen; i++)
    {
        send[0].Data[i] = data[i];
    }
    return send[0];
}

//打印控制消息
void Show_Transmit(VCI_CAN_OBJ send){
    printf("Index:%04d  ",Send_count);Send_count++;
    printf("CAN2 TX ID:0x%08X", send.ID);
    if(send.ExternFlag==0) printf(" Standard ");
    if(send.ExternFlag==1) printf(" Extend   ");
    if(send.RemoteFlag==0) printf(" Data   ");
    if(send.RemoteFlag==1) printf(" Remote ");
    printf("DLC:0x%02X",send.DataLen);
    printf(" data:0x");
    for(int i = 0; i < send.DataLen; i++)
    {
        printf(" %02X", send.Data[i]);
    }
    printf("\n");

}

//关闭CAN线程
void CAN_Close(){
//    m_run0=0;//线程关闭指令。
//    pthread_join(threadid,NULL);//等待线程关闭。
    usleep(100000);//延时100ms。
    VCI_ResetCAN(VCI_USBCAN2, 0, 0);//复位CAN1通道。
    usleep(100000);//延时100ms。
    VCI_ResetCAN(VCI_USBCAN2, 0, 1);//复位CAN2通道。
    usleep(100000);//延时100ms。
    int ret1 = VCI_CloseDevice(VCI_USBCAN2,0);//关闭设备。
    if(ret1 == 1)
        cout << " ret1 = " << ret1 << endl << "关闭成功" << endl;
}

float Axle_spacing;
float Wheel_spacing;

float Vz_to_Akm_Angle(float Vx, float Vz)
{
    float R, AngleR, Min_Turn_Radius;
//float AngleL;
//阿克曼小车需要设置最小转弯半径
//如果目标速度要求的转弯半径小于最小转弯半径，
//会导致小车运动摩擦力大大提高，严重影响控制效果

    if(Vz!=0 && Vx!=0)
    {
//如果目标速度要求的转弯半径小于最小转弯半径
        if(fabs(Vx/Vz)<=Min_Turn_Radius)
        {
//降低目标角速度，配合前进速度，提高转弯半径到最小转弯半径
            if(Vz>0)
                Vz= Vx/(Min_Turn_Radius);
            else
                Vz=-Vx/(Min_Turn_Radius);
        }
        R=Vx/Vz;
//AngleL=atan(Axle_spacing/(R-0.5*Wheel_spacing));
        AngleR=atan(Axle_spacing/(R+0.5*Wheel_spacing));
    }
    else
    {
        AngleR=0;
    }
    return AngleR;
}














