/*
 *该文件目前用于存放user层除main.c外需要用到的变量定义
 */
#include "variable.h"

/*IMU*/
IMU_Data imu_data;


/*ROS 下发的PID参数*/ //最初pid参数的设定，用于上位机以发布话题的形式调参
float ros_angle_kp = 67.5;//54;//90;
float ros_angle_kd = 3.0;//2.4;//4.0;
float ros_speed_kp = 400;//380;
float ros_speed_ki = 2.0;//1.9;
float ros_direction_kp = 100;
float ros_direction_ki = 0;

/*目标速度*/
float goal_velocity[WHEEL_NUM] = {0.0, 0.0};

int left_encoder_count, right_encoder_count;

/*车模跌倒标志位*/
bool g_bFallFlag = 0;

/*蓝牙控制相关变量定义*/
float g_fBTSpeedSet = 0.0;
float g_fBTDirectionSet = 0.0;

/*虚拟示波器相关变量定义*/
float g_fware[8];