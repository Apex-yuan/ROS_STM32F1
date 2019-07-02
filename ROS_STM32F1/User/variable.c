/*
 *���ļ�Ŀǰ���ڴ��user���main.c����Ҫ�õ��ı�������
 */
#include "variable.h"

/*IMU*/
IMU_Data imu_data;


/*ROS �·���PID����*/ //���pid�������趨��������λ���Է����������ʽ����
float ros_angle_kp = 67.5;//54;//90;
float ros_angle_kd = 3.0;//2.4;//4.0;
float ros_speed_kp = 400;//380;
float ros_speed_ki = 2.0;//1.9;
float ros_direction_kp = 100;
float ros_direction_ki = 0;

/*Ŀ���ٶ�*/
float goal_velocity[WHEEL_NUM] = {0.0, 0.0};

int left_encoder_count, right_encoder_count;

/*��ģ������־λ*/
bool g_bFallFlag = 0;

/*����������ر�������*/
float g_fBTSpeedSet = 0.0;
float g_fBTDirectionSet = 0.0;

/*����ʾ������ر�������*/
float g_fware[8];