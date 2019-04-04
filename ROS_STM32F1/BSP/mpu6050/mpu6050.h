#ifndef __MPU6050_H__
#define __MPU6050_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f10x.h"

#define DEFAULT_MPU_HZ  (100) //DMP输出速率，最大为200HZ

//MPU6050 DMP固件 加速度计的默认范围选择为：+/-2(g)   陀螺仪的默认范围选择为：+/- 2000(deg/s) 具体配置在mpu_init()函数中。
  //mpu6050加速度计和陀螺仪范围和精度详细资料参考：http://blog.csdn.net/u013636775/article/details/69668860
#define ACCEL_FACTOR       0.000598550415   // (ADC_Value / Scale) * 9.80665            => Range : +- 2[g] （选择的量程）
                                            //                                             Scale : +- 16384 （该量程对应的精度）
#define GYRO_FACTOR        0.0010642        // (ADC_Value/Scale) * (pi/180)（单位：rad/s） => Range : +- 2000[deg/s] （选择的量程）
                                            //                                                Scale : +- 16.4[deg/s]  （该量程对应的精度）
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
