#ifndef __CONFIG_H
#define __CONFIG_H

#ifdef __cplusplus
 extern "C" {
#endif

#define CMD_VEL_PUBLISH_FREQUENCY  30 //hz
#define DRIVE_INFORMATION_PUBLISH_FREQUENCY  30 //hz
#define IMU_PUBLISH_FREQUENCY                1000 //hz

//机器人底盘信息   
#define WHEEL_NUM  2   
#define WHEEL_RADIUS   0.0034 //0.033 //meter
#define WHEEL_SEPARATION 0.218 //0.160  //meter
   
#define LINEAR   0
#define ANGULAR  1
   
#define LEFT 0
#define RIGHT 1

#define DEG2RAD(x)   (x * 0.01745329252) // *PI/180
#define RAD2DEG(x)   (x * 57.2957795131) // *180/PI   

//脉冲数转为对应弧度的乘数因子：rad = tick * TICK2RAD
//轮子转动一圈的脉冲数：13（编码器线数） * 4（一个周期脉冲捕捉次数） * 30（减速比） = 1560
//单个脉冲对应的弧度即 TICK2RAD = 1 / 1560 * (2*PI)
#define TICK2RAD 0.004027683  //    // 0.087890625[deg] * 3.14159265359 / 180 = 0.001533981   
     
   
   
#ifdef __cplusplus
 }
#endif

#endif /*__CONFIG_H */

