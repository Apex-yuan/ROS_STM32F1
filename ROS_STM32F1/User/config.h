#ifndef __CONFIG_H
#define __CONFIG_H

#ifdef __cplusplus
 extern "C" {
#endif

#define CMD_VEL_PUBLISH_FREQUENCY  30 //hz
#define DRIVE_INFORMATION_PUBLISH_FREQUENCY  30 //hz
#define IMU_PUBLISH_FREQUENCY                1000 //hz

//�����˵�����Ϣ   
#define WHEEL_NUM  2   
#define WHEEL_RADIUS   0.0034 //0.033 //meter
#define WHEEL_SEPARATION 0.218 //0.160  //meter
   
#define LINEAR   0
#define ANGULAR  1
   
#define LEFT 0
#define RIGHT 1

#define DEG2RAD(x)   (x * 0.01745329252) // *PI/180
#define RAD2DEG(x)   (x * 57.2957795131) // *180/PI   

//������תΪ��Ӧ���ȵĳ������ӣ�rad = tick * TICK2RAD
//����ת��һȦ����������13�������������� * 4��һ���������岶׽������ * 30�����ٱȣ� = 1560
//���������Ӧ�Ļ��ȼ� TICK2RAD = 1 / 1560 * (2*PI)
#define TICK2RAD 0.004027683  //    // 0.087890625[deg] * 3.14159265359 / 180 = 0.001533981   
     
   
   
#ifdef __cplusplus
 }
#endif

#endif /*__CONFIG_H */

