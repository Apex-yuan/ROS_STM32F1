/*
 *���ļ�Ŀǰ���ڴ��user���main.c����Ҫ�õ��ı�������
 */
#include "variable.h"

/*IMU*/
IMU_Data imu_data;


/*ROS �·���PID����*/ //���pid�������趨��������λ���Է����������ʽ����
float ros_angle_kp = 60;//60;
float ros_angle_kd = 3;//0.7;
float ros_speed_kp = 350;//300;
float ros_speed_ki = 1.5;//5;
float ros_direction_kp = 200;
float ros_direction_ki = 0.5;

/*Ŀ���ٶ�*/
float goal_velocity[WHEEL_NUM] = {0.0, 0.0};

int left_encoder_count, right_encoder_count;


/*����������ر�������*/
float g_fBTSpeedSet = 0.0;
float g_fBTDirectionSet = 0.0;

/*����ʾ������ر�������*/
float g_fware[8];