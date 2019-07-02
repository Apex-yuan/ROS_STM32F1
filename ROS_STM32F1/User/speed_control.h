#ifndef __SPEED_CONTROL_H
#define __SPEED_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "config.h"

#define OPTICAL_ENCODE_CONSTANT  (13*4) //�������ֱ���
#define REDUCTION_RATIO          30 //���ٱ�
#define SPEED_CONTROL_PERIOD  100  //�ٶȿ������ڣ�ms��
#define CAR_SPEED_CONSTANT  (1000.0/SPEED_CONTROL_PERIOD/OPTICAL_ENCODE_CONSTANT/REDUCTION_RATIO)  //��λת������ֵ�����ٶȵ�λת��Ϊ ת/�룬��Ӧ����ת�٣�

#define SPEED_CONTROL_COUNT (SPEED_CONTROL_PERIOD / CONTROL_PERIOD) //20*5=100ms  
  
#define MPS2NPS  ((float)1/(2*PI*WHEEL_RADIUS))

#define CAR_SPEED_SET 0
#define MOTOR_OUT_DEAD_VAL 0

extern float g_fCarSpeed;
extern int g_nLeftMotorPulseSigma, g_nRightMotorPulseSigma;
extern float g_fSpeedControlOutOld, g_fSpeedControlOutNew;
extern float g_fSpeedControlIntegral;
extern float g_fSpeedControlOut;
extern uint16_t g_nSpeedControlPeriod;
extern float g_nSpeedControlCount;


void GetMotorPulse(void);
void SpeedControl(void);
void SpeedControlOutput(void);

#ifdef __cplusplus
}
#endif

#endif /* __SPEED_CONTROL_H */

