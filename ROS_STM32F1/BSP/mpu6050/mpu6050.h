#ifndef __MPU6050_H__
#define __MPU6050_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f10x.h"

#define DEFAULT_MPU_HZ  (100) //DMP������ʣ����Ϊ200HZ

//MPU6050 DMP�̼� ���ٶȼƵ�Ĭ�Ϸ�Χѡ��Ϊ��+/-2(g)   �����ǵ�Ĭ�Ϸ�Χѡ��Ϊ��+/- 2000(deg/s) ����������mpu_init()�����С�
  //mpu6050���ٶȼƺ������Ƿ�Χ�;�����ϸ���ϲο���http://blog.csdn.net/u013636775/article/details/69668860
#define ACCEL_FACTOR       0.000598550415   // (ADC_Value / Scale) * 9.80665            => Range : +- 2[g] ��ѡ������̣�
                                            //                                             Scale : +- 16384 �������̶�Ӧ�ľ��ȣ�
#define GYRO_FACTOR        0.0010642        // (ADC_Value/Scale) * (pi/180)����λ��rad/s�� => Range : +- 2000[deg/s] ��ѡ������̣�
                                            //                                                Scale : +- 16.4[deg/s]  �������̶�Ӧ�ľ��ȣ�
#define q30  1073741824.0f
  
int8_t MPU_I2C_Read(uint8_t addr, uint8_t reg, uint8_t len, uint8_t * data);
int8_t MPU_I2C_Write(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);

uint8_t MPU_DMP_Init(void);
uint8_t MPU_DMP_ReadData(float *gyro, float *accel ,float *quat, float *rpy);
//uint8_t MPU_DMP_GetData(short *gyro,short *accel ,float *pitch,float *roll,float *yaw);

#ifdef __cplusplus
 }
#endif
  
#endif
