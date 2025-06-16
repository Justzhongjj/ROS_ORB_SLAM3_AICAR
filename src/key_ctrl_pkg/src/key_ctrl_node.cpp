
#include "../include/key_ctrl_pkg/common.h"
#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "../include/key_ctrl_pkg/key.h"

const int KEYCODE_W = 0x77;
const int KEYCODE_A = 0x61;
const int KEYCODE_S = 0x73;
const int KEYCODE_D = 0x64;
 
const int KEYCODE_A_CAP = 0x41;
const int KEYCODE_D_CAP = 0x44;
const int KEYCODE_S_CAP = 0x53;
const int KEYCODE_W_CAP = 0x57;
 
const int KEYCODE_0 = 0x30;
const int KEYCODE_9 = 0x39;
 
const int KEYCODE_z_P = 0x7A;
const int KEYCODE_x_D = 0x78;
const int KEYCODE_c_R = 0x63;
const int KEYCODE_v_N = 0x76;
const int KEYCODE_z_CAP_P = 0x5A;
const int KEYCODE_x_CAP_D = 0x58;
const int KEYCODE_c_CAP_R = 0x43;
const int KEYCODE_v_CAP_N = 0x56;
 
const int KEYCODE_BACKSPACE = 0x7F;
// const int KEYCODE_SHIFT = 0x10;
// const int KEYCODE_CTRL = 0x11;
// const int KEYCODE_ALT = 0x11;
const int KEYCODE_ESC = 0x1B;
const int KEYCODE_ENTER = 0x0A;
const int KEYCODE_SPACE = 0x20;
 
using namespace KEY_CTRL;
//using namespace CONTROL_COMMON;
int main(int argc, char **argv) {
  ros::init(argc, argv, "keyboard_control");
  ros::NodeHandle nh;
 
  ros::Publisher pub_keyboard =
      nh.advertise<std_msgs::Int32>("/keyboard_ctrl", 1000);
      //nh.advertise<control_msgs::keyboard_ctrl>("/keyboard_ctrl", 1000);
  
  
  printf("*****************Menu*****************\n\n");
  printf("SPACE: start\n");
  printf(" 9 : stop\n");
  printf(" 0 : close\n");
  printf("W/w: up\n");
  printf("S/s: down\n");
  printf("A/a: left\n");
  printf("D/d: right\n");
  printf("**************************************\n\n");
  
  ros::Rate loop_rate(1000);

  auto KBC = Keyboard_ctrl();
  while (ros::ok()) {
    auto key = KBC.get_keyboard_press_key();
    ROS_INFO("get keyboard press 0x%02X \n", key);
 
   // static GEER geer_mode = GEER::P;
    int value = 0;
    KEY_CLASS order;
 
    /*
        if (key >= KEYCODE_0 && key <= KEYCODE_9) {
      order = KEY_CLASS::SPEED;
      value = key - KEYCODE_0;
 
      ROS_INFO("set car speed : %d ", value);
 
    }
    */
    
    if (key == KEYCODE_SPACE) {
      value = int(KB::START);
      ROS_INFO("start car: %d ", value);
    } else if (key == KEYCODE_0) {
      value = int(KB::CLOSE);
      ROS_INFO("close car: %d", value);
    } else if (key == KEYCODE_9) {
      value = int(KB::STOP);
      ROS_INFO("stop car: %d", value);
    }else if (key == KEYCODE_W || key == KEYCODE_W_CAP) {
      value = int(KB::UP);
      ROS_INFO("manu ctrl up %d", value);
    }  else if (key == KEYCODE_A || key == KEYCODE_A_CAP) {
      value = int(KB::LEFT);
      ROS_INFO("manu ctrl left %d", value);
    } else if(key == KEYCODE_D || key == KEYCODE_D_CAP) {
      value = int(KB::RIGHT);
      ROS_INFO("manu ctrl right %d", value);
    } else if (key == KEYCODE_S || key == KEYCODE_S_CAP) {
      value = int(KB::DOWN);
      ROS_INFO("manu ctrl down %d", value);
    }
 
    //control_msgs::keyboard_ctrl kbc_msg;
    std_msgs::Int32 kbc_msg;
    kbc_msg.data=value;
    //kbc_msg.data.push_back(value);
    pub_keyboard.publish(kbc_msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
