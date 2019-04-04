
/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "HardwareSerial.h"
#include "config.h"

#include <ros.h>
#include <geometry_msgs/Twist.h> 
#include <std_msgs/Empty.h>
#include "systick.h"
#include "led.h"

void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg);
void ledCallback(const std_msgs::Empty& led_msg);

/* ROS NodeHandle ------------------------------------------------------------------*/
ros::NodeHandle nh;

/* Suberscriber ------------------------------------------------------------------*/
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel",commandVelocityCallback);
ros::Subscriber<std_msgs::Empty > led_sub("toggle_led", ledCallback);


uint32_t t;
static uint32_t tTime[10];
float goal_velocity_from_cmd[WHEEL_NUM] = {0.0, 0.0};


void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg)
{
  goal_velocity_from_cmd[LINEAR] = cmd_vel_msg.linear.x;
  goal_velocity_from_cmd[ANGULAR] = cmd_vel_msg.angular.z;
}

void ledCallback(const std_msgs::Empty& led_msg)
{
  GPIO_WriteBit(GPIOA, GPIO_Pin_8, (BitAction)(1 - GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_8)));
}

int main()
{
	systick_init();	//滴答定时器初始化
  Serial.begin(57600);
  led_init();
 
  nh.initNode();

  nh.loginfo("connected seccess!");
  nh.subscribe(cmd_vel_sub);
  nh.subscribe(led_sub);

  
  while(1)
  {
    t = millis();
    if((t - tTime[0]) > (1000 / CMD_VEL_PUBLISH_FREQUENCY))
    {
      
    }      
    nh.spinOnce();
    //delay(1000);
  }
}