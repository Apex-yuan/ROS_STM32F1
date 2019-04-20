#ifndef __MAIN_H
#define __MAIN_H

#include <ros.h>
#include <ros/time.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/BatteryState.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Float32MultiArray.h>


ros::Time addMicros(ros::Time & t, uint32_t _micros);
void updateTime(); 
ros::Time rosNow();

void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg);
void ledCallback(const std_msgs::Empty& led_msg);
void pidCallback(const std_msgs::Float32MultiArray & pid_msg);

void initSonar(void);
void initRpy(void);
void initOdom(void);
void initJointStates(void);

void publishSonarMsg(void);
void publishRpyMsg(void);
void publishBatteryStateMsg(void);
void publishImuMsg(void);
void publishDriveInformation(void);
bool calcOdometry(double diff_time);
void updateMotorInfo(int16_t left_tick, int16_t right_tick);
void updateOdometry(void);
void updateJointStates(void);
void updateTF(geometry_msgs::TransformStamped& odom_tf);
void updateGoalVelocity(void);
void motorControl(float linear_vel, float angular);

void sendLogMsg(void);


#endif /*__MAIN_H*/
