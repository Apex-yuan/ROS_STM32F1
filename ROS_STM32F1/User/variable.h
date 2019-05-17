#ifndef __VARIABLE_H
#define __VARIABLE_H

#include "mpu6050.h"
#include "config.h"

extern IMU_Data imu_data;

extern float ros_angle_kp;
extern float ros_angle_kd;
extern float ros_speed_kp;
extern float ros_speed_ki;
extern float ros_direction_kp;
extern float ros_direction_ki;

extern float goal_velocity[WHEEL_NUM];
extern int left_encoder_count, right_encoder_count;

extern float g_fBTSpeedSet;
extern float g_fBTDirectionSet;

extern float g_fware[8];

#endif /* __VARIABLE_H*/
