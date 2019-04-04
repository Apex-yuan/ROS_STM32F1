#include <stdio.h>
#include "HardwareSerial.h"

#include <ros.h>
#include <std_msgs/String.h>
#include "systick.h"

ros::NodeHandle  nh;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

char hello[13] = "hello world!";


int main()
{
	systick_init();							//延时函数初始化*/
  
  nh.initNode();
  nh.advertise(chatter);

  while(1)
  {
    //Serial.putstr("dfisdhfs\n");
    Serial.write('s');
    str_msg.data = hello;
    chatter.publish( &str_msg );
    nh.spinOnce();
    delay(1000);
  }
}
