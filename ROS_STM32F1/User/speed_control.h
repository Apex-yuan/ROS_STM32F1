#ifndef __SPEED_CONTROL_H
#define __SPEED_CONTROL_H

#include <stdint.h>
//#include "stm32f10x.h"

#define OPTICAL_ENCODE_CONSTANT  (13*4) //�������ֱ���
#define REDUCTION_RATIO          30 //���ٱ�
#define SPEED_CONTROL_PERIOD  100  //�ٶȿ������ڣ�ms��
#define CAR_SPEED_CONSTANT  (1000.0/SPEED_CONTROL_PERIOD/OPTICAL_ENCODE_CONSTANT/REDUCTION_RATIO)  //��λת������ֵ�����ٶȵ�λת��Ϊ ת/�룬��Ӧ����ת�٣�

#define CAR_SPEED_SET 0
#define MOTOR_OUT_DEAD_VAL 0

extern float g_fCarSpeed;
extern int g_nLeftMotorPulseSigma, g_nRightMotorPulseSigma;
extern float g_fSpeedControlOutOld, g_fSpeedControlOutNew;
extern float g_fSpeedControlIntegral;
extern float g_fSpeedControlOut;
extern uint16_t g_nSpeedControlPeriod;

//����������ر���
extern float g_fBTSpeedSet;


void GetMotorPulse(void);
void SpeedControl(void);
void SpeedControlOutput(void);

#endif /* __SPEED_CONTROL_H */

