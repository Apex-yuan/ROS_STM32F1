
/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "HardwareSerial.h"
#include "config.h"

#include <ros.h>
#include <ros/time.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/Imu.h>


#include "systick.h"
#include "led.h"
#include "encoder.h"
#include "i2c.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "motor.h"
#include "stm32f10x.h"

void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg);
void ledCallback(const std_msgs::Empty& led_msg);

void initOdom(void);
void initJointStates(void);

void publishImuMsg(void);
void publishDirveInformation(void);
bool calcOdometry(double diff_time);
void updateMotorInfo(int16_t left_tick, int16_t right_tick);
void updateOdometry(void);
void updateJointStates(void);
void updateTF(geometry_msgs::TransformStamped& odom_tf);
void updateGoalVelocity(void);
void motorControl(float linear_vel, float angular);

void sendLogMsg(void);

/* ROS NodeHandle ------------------------------------------------------------------*/
ros::NodeHandle nh;

/* Publisher ---------------------------------------------------------------------*/
//Odometry
nav_msgs::Odometry odom;
ros::Publisher odom_pub("odom", &odom);
//Joint (Dynamixel) state of robot
sensor_msgs::JointState joint_states;
ros::Publisher joint_states_pub("joint_states",&joint_states);
//IMU 
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu", &imu_msg);

/* Tranceform Broadcaster ----------------------------------------------------------*/
geometry_msgs::TransformStamped tfs_msg;
geometry_msgs::TransformStamped odom_tf;
tf::TransformBroadcaster tf_broadcaster;

/* Suberscriber ------------------------------------------------------------------*/
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel",commandVelocityCallback);
ros::Subscriber<std_msgs::Empty > led_sub("toggle_led", ledCallback);

uint32_t t;
static uint32_t tTime[10];
float goal_velocity[WHEEL_NUM] = {0.0, 0.0};
float goal_velocity_from_cmd[WHEEL_NUM] = {0.0, 0.0};

char odom_header_frame_id[30] = "/odom";
char odom_child_frame_id[30] = "/base_footprint";

char joint_state_header_frame_id[30];

//record imu data
float gyro[3];  // rad/s
float accel[3]; // m/s^2
float quat[4];   // float format
float rpy[3];   // rad

//caculate for odometry
bool init_encoder = true;
int16_t last_diff_tick[WHEEL_NUM] = {0.0, 0.0};
double last_rad[WHEEL_NUM] = {0.0, 0.0};

//update joint state
double last_velocity[WHEEL_NUM] = {0.0, 0.0};

//declatation for SLAM and navigation
uint32_t prev_update_time;
float odom_pose[3];
float odom_vel[3];

//PID
float left_control_output, left_control_output_old;
float right_control_output, right_control_output_old;
float linear_control_output, linear_control_output_old;
float linear_kp = -1000;
float linear_ki = -800;
float angular_control_output, angular_control_output_old;
float angular_kp = 300;
float angular_ki = 0;

void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg)
{
  goal_velocity_from_cmd[LINEAR] = cmd_vel_msg.linear.x;
  goal_velocity_from_cmd[ANGULAR] = cmd_vel_msg.angular.z;
}

void ledCallback(const std_msgs::Empty& led_msg)
{
  GPIO_WriteBit(GPIOA, GPIO_Pin_8, (BitAction)(1 - GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_8)));
}



int main()
{
  
  
	systick_init();	//滴答定时器初始化
  Serial.begin(57600);
  led_init();
  MotorInit();
  EncoderInit();
  
  while(MPU_DMP_Init()); //等待IMU初始化完成
 
  nh.initNode();
  //nh.getHardware()->setBaud(115200);

  nh.loginfo("connected seccess!");
  nh.subscribe(cmd_vel_sub);
  nh.subscribe(led_sub);

  nh.advertise(imu_pub);
  nh.advertise(odom_pub);
  nh.advertise(joint_states_pub);
  
  tf_broadcaster.init(nh);
  
  initOdom();
  initJointStates();
  
  prev_update_time = millis();
  
  while(1)
  {
    GPIO_SetBits(GPIOC, GPIO_Pin_13);
    t = millis();
    if((t - tTime[0]) > (1000 / CMD_VEL_PUBLISH_FREQUENCY))
    {
      
      updateGoalVelocity();
      motorControl(goal_velocity[LINEAR], goal_velocity[ANGULAR]);
      tTime[0] = t;
    }  
    if((t - tTime[2]) > (1000 / DRIVE_INFORMATION_PUBLISH_FREQUENCY))
    {
      publishDirveInformation();  
      tTime[2] = t;
    }    
    if((t - tTime[3]) > (1000 / IMU_PUBLISH_FREQUENCY))
    {
      publishImuMsg();
      tTime[3] = t;
    }
    if((t - tTime[4]) > (1000 / 4))
    {
      if(nh.connected())
      {
        GPIO_ResetBits(GPIOC, GPIO_Pin_13);
      }
      tTime[4] = t;
    }
    else
      GPIO_SetBits(GPIOC, GPIO_Pin_13);
    
    // Send log message after ROS connection
    sendLogMsg();

    MPU_DMP_ReadData(gyro, accel, quat, rpy);
    

    nh.spinOnce();
    
    //delay_ms(10);
  }
}


//
void initOdom(void)
{
  init_encoder = true;
  
  for(int index = 0; index < 3; index++)
  {
    odom_pose[index] = 0.0;
    odom_vel[index] = 0.0;
  }
  odom.pose.pose.position.x = 0.0;
  odom.pose.pose.position.y = 0.0;
  odom.pose.pose.position.z = 0.0;
  
  odom.pose.pose.orientation.x = 0.0;
  odom.pose.pose.orientation.y = 0.0;
  odom.pose.pose.orientation.z = 0.0;
  odom.pose.pose.orientation.w = 0.0;
  
  odom.twist.twist.linear.x = 0.0;
  odom.twist.twist.angular.z = 0.0;
  
}

void initJointStates(void)
{
  static char *joint_states_name[] = {"wheel_left_joint", "wheel_right_joint"};
  
  joint_states.header.frame_id = joint_state_header_frame_id;
  joint_states.name            = joint_states_name;
  
  joint_states.name_length     = WHEEL_NUM;
  joint_states.position_length = WHEEL_NUM;
  joint_states.velocity_length = WHEEL_NUM;
  joint_states.effort_length   = WHEEL_NUM;
}

//Publish msgs (IMU data:angular velocity, linear acceleration, orientation)
void publishImuMsg(void)
{
  imu_msg.header.stamp  = nh.now();
  imu_msg.header.frame_id = "imu_link";
  
  imu_msg.angular_velocity.x = gyro[0];
  imu_msg.angular_velocity.y = gyro[1];
  imu_msg.angular_velocity.z = gyro[2];
  imu_msg.angular_velocity_covariance[0] = 0.02;
  imu_msg.angular_velocity_covariance[1] = 0;
  imu_msg.angular_velocity_covariance[2] = 0;
  imu_msg.angular_velocity_covariance[3] = 0;
  imu_msg.angular_velocity_covariance[4] = 0.02;
  imu_msg.angular_velocity_covariance[5] = 0;
  imu_msg.angular_velocity_covariance[6] = 0;
  imu_msg.angular_velocity_covariance[7] = 0;
  imu_msg.angular_velocity_covariance[8] = 0.02;
  
  imu_msg.linear_acceleration.x = accel[0];
  imu_msg.linear_acceleration.y = accel[1];
  imu_msg.linear_acceleration.z = accel[2];
  imu_msg.linear_acceleration_covariance[0] = 0.04;
  imu_msg.linear_acceleration_covariance[1] = 0;
  imu_msg.linear_acceleration_covariance[2] = 0;
  imu_msg.linear_acceleration_covariance[3] = 0;
  imu_msg.linear_acceleration_covariance[4] = 0.04;
  imu_msg.linear_acceleration_covariance[5] = 0;
  imu_msg.linear_acceleration_covariance[6] = 0;
  imu_msg.linear_acceleration_covariance[7] = 0;
  imu_msg.linear_acceleration_covariance[8] = 0.04;
  
  imu_msg.orientation.w = quat[0];
  imu_msg.orientation.x = quat[1];
  imu_msg.orientation.y = quat[2];
  imu_msg.orientation.z = quat[3];
  
  imu_msg.orientation_covariance[0] = 0.0025;
  imu_msg.orientation_covariance[1] = 0;
  imu_msg.orientation_covariance[2] = 0;
  imu_msg.orientation_covariance[3] = 0;
  imu_msg.orientation_covariance[4] = 0.0025;
  imu_msg.orientation_covariance[5] = 0;
  imu_msg.orientation_covariance[6] = 0;
  imu_msg.orientation_covariance[7] = 0;
  imu_msg.orientation_covariance[8] = 0.0025;
  
  imu_pub.publish(&imu_msg);
  
  tfs_msg.header.stamp   = nh.now();
  tfs_msg.header.frame_id = "base_link";
  tfs_msg.child_frame_id = "imu_link";
  tfs_msg.transform.rotation.w = quat[0];
  tfs_msg.transform.rotation.x = quat[1];
  tfs_msg.transform.rotation.y = quat[2];
  tfs_msg.transform.rotation.z = quat[3];
  
  tfs_msg.transform.translation.x = 0.0;
  tfs_msg.transform.translation.y = 0.0;
  tfs_msg.transform.translation.z = 0.068;
  tf_broadcaster.sendTransform(tfs_msg);
}

int16_t encoder_l, encoder_r;
void publishDirveInformation(void)
{
  unsigned long time_now = millis();
  unsigned long step_time = time_now - prev_update_time;
  //int16_t encoder_l, encoder_r;
  
  prev_update_time = time_now;
  ros::Time stamp_now = nh.now(); //rosNow();
  
  encoder_l = (int16_t)TIM_GetCounter(TIM3);
  encoder_r = - (int16_t)TIM_GetCounter(TIM4);
  
  updateMotorInfo(encoder_l, encoder_r);
  
  calcOdometry((double)(step_time * 0.001));
  
  //odometry
  updateOdometry();
  odom.header.stamp = stamp_now;
  odom_pub.publish(&odom);
  
  //tf
  updateTF(odom_tf);
  odom_tf.header.stamp = stamp_now;
  tf_broadcaster.sendTransform(odom_tf);
  
  //joint states
  updateJointStates();
  joint_states.header.stamp = stamp_now;
  joint_states_pub.publish(&joint_states);
}

/*******************************************************************************
* Update motor information
*******************************************************************************/
void updateMotorInfo(int16_t left_tick, int16_t right_tick)
{
  int16_t current_tick = 0;
  static int16_t last_tick[WHEEL_NUM] = {0.0, 0.0};
  
//  if (init_encoder)
//  {
//    for (int index = 0; index < WHEEL_NUM; index++)
//    {
//      last_diff_tick[index] = 0.0;
//      last_tick[index]      = 0.0;
//      last_rad[index]       = 0.0;

//      last_velocity[index]  = 0.0;
//    }  

//    last_tick[LEFT] = left_tick;
//    last_tick[RIGHT] = right_tick;

//    init_encoder = false;
//    return;
//  }

  current_tick = left_tick;

  last_diff_tick[LEFT] = current_tick - last_tick[LEFT];
  last_tick[LEFT]      = current_tick;
  last_rad[LEFT]       += TICK2RAD * (double)last_diff_tick[LEFT];

  current_tick = right_tick;

  last_diff_tick[RIGHT] = current_tick - last_tick[RIGHT];
  last_tick[RIGHT]      = current_tick;
  last_rad[RIGHT]       += TICK2RAD * (double)last_diff_tick[RIGHT];
}

bool calcOdometry(double diff_time)
{
  double wheel_l, wheel_r;   //rotation value of wheel [rad]
  double delta_s, theta, delta_theta;
  static double last_theta = 0.0;
  double v,w;   //v = translational velocity [m/s], w = rotational velocity [rad/s]
  double step_time;
  
  wheel_l = wheel_r = 0;
  delta_s = delta_theta = theta = 0.0;
  v = w = 0.0;
  step_time = 0.0;
  
  step_time = diff_time;
  
  if(step_time == 0)
    return false;
  
  wheel_l = TICK2RAD * (double)last_diff_tick[LEFT];
  wheel_r = TICK2RAD * (double)last_diff_tick[RIGHT];
  
  delta_s  = WHEEL_RADIUS * (wheel_l + wheel_r) / 2;
  //通过odom数据计算偏转角raw
  theta = WHEEL_RADIUS * (wheel_r - wheel_l) / WHEEL_SEPARATION;
  delta_theta = theta - last_theta;
  //通过IMU数据计算偏转角raw
  //theta = atan2f((quat[1] * quat[2] + quat[0] * quat[3]), 0.5f - quat[2] * quat[2] - quat[3] * quat[3]);
  //delta_theta = theta - last_theta;
  
  //compute odometric pose
  odom_pose[0] += delta_s * cos(odom_pose[2] + (delta_theta / 2.0));
  odom_pose[1] += delta_s * sin(odom_pose[2] + (delta_theta / 2.0));
  odom_pose[2] += theta;//delta_theta;
  
  //compute odometric instantaneouse velocity
  v = delta_s / step_time;
  w = theta / step_time;
  
  odom_vel[0] = v;
  odom_vel[1] = 0.0;
  odom_vel[2] = w;
  
  last_velocity[LEFT] = wheel_l / step_time;
  last_velocity[RIGHT] = wheel_r / step_time;
  last_theta = theta;
  
  return true;
}

void updateOdometry(void)
{
  odom.header.frame_id = "odom"; //odom_header_frame_id;
  odom.child_frame_id = "base_link"; //odom_child_frame_id;
  
  odom.pose.pose.position.x = odom_pose[0];
  odom.pose.pose.position.y = odom_pose[1];
  odom.pose.pose.position.z = 0;
  odom.pose.pose.orientation = tf::createQuaternionFromYaw(odom_pose[2]);
  
  odom.twist.twist.linear.x = odom_vel[0];
  odom.twist.twist.angular.z = odom_vel[2];
}

void updateJointStates(void)
{
  static float joint_states_pos[WHEEL_NUM] = {0.0, 0.0};
  static float joint_states_vel[WHEEL_NUM] = {0.0, 0.0};
  //static float joint_states_eff[WHEEL_NUM] = {0.0, 0.0};
  
  joint_states_pos[LEFT] = last_rad[LEFT];
  joint_states_pos[RIGHT] = last_rad[RIGHT];
  
  joint_states_vel[LEFT] = last_velocity[LEFT];
  joint_states_vel[RIGHT] = last_velocity[RIGHT];
  
  joint_states.position = joint_states_pos;
  joint_states.velocity = joint_states_vel;
}

void updateTF(geometry_msgs::TransformStamped& odom_tf)
{
  odom_tf.header = odom.header;
  odom_tf.child_frame_id = "base_footprint"; //odom.child_frame_id;
  odom_tf.transform.translation.x = odom.pose.pose.position.x;
  odom_tf.transform.translation.y = odom.pose.pose.position.y;
  odom_tf.transform.translation.z = odom.pose.pose.position.z;
  odom_tf.transform.rotation = odom.pose.pose.orientation;
}

void updateGoalVelocity(void)
{
  goal_velocity[LINEAR] = goal_velocity_from_cmd[LINEAR];
  goal_velocity[ANGULAR] = goal_velocity_from_cmd[ANGULAR];  
}

float left_speed_control_integral;
float right_speed_control_integral;

//目前的PID计算函数还存在一些问题，后续修改。　2019/3/30
float leftPIDCaculate(float goal_vel, float real_vel, float kp, float ki)
{
  float delta_vel;
  float fP,fI;
  static float speed_control_integral;
  
  delta_vel = goal_vel - real_vel;
   
  fP = delta_vel * kp;
  fI = delta_vel * ki;
  
  left_speed_control_integral += fI;
  
  left_control_output_old = left_control_output;
  left_control_output = fP + left_speed_control_integral;
  return left_control_output;
}
float rightPIDCaculate(float goal_vel, float real_vel, float kp, float ki)
{
  float delta_vel;
  float fP,fI;
  static float speed_control_integral;
  
  delta_vel = goal_vel - real_vel;
   
  fP = delta_vel * kp;
  fI = delta_vel * ki;
  
  right_speed_control_integral += fI;
  
  right_control_output_old = right_control_output;
  right_control_output = fP + right_speed_control_integral;
  return right_control_output;
}

void linearPIDCaculate(float goal_linear_vel)
{
  float delta_vel;
  float fP,fI;
  static float speed_control_integral;
  
  delta_vel = goal_linear_vel - odom_vel[0];//last_velocity[LEFT] * WHEEL_RADIUS;//odom_vel[0];
   
  fP = delta_vel * linear_kp;
  fI = delta_vel * linear_ki;
  
  speed_control_integral += fI;
  
  linear_control_output_old = linear_control_output;
  linear_control_output = fP + speed_control_integral;
}

void angularPIDCaculate(float goal_angular_vel)
{
  float delta_vel;
  float fP,fI;
  static float speed_control_integral;
  
  delta_vel = goal_angular_vel - odom_vel[2];
   
  fP = delta_vel * angular_kp;
  fI = delta_vel * angular_ki;
  
  speed_control_integral += fI;
  
  angular_control_output_old = angular_control_output;
  angular_control_output = fP + speed_control_integral;
}
float wheel_vel[WHEEL_NUM], wheel_vel_old[WHEEL_NUM];
uint16_t motor_output[WHEEL_NUM];
// left motor:  gpio : PB13,PB12   PWM: PA3
// right motor: gpio : PB14,PB15   PWM: PA2
void motorControl(float linear_vel, float angular_vel)
{
  //float wheel_vel[WHEEL_NUM];
  //uint16_t motor_output[WHEEL_NUM];
  float left_out, right_out;
  
  //linearPIDCaculate(linear_vel);
  //angularPIDCaculate(angular_vel);
  
  wheel_vel[LEFT]  = linear_vel - (angular_vel * WHEEL_SEPARATION / 2);
  wheel_vel[RIGHT] = linear_vel + (angular_vel * WHEEL_SEPARATION / 2);

  wheel_vel_old[LEFT] = odom_vel[0] - (odom_vel[2] * WHEEL_SEPARATION / 2);
  wheel_vel_old[RIGHT] = odom_vel[0] + (odom_vel[2] * WHEEL_SEPARATION / 2);  
  
  //wheel_vel[LEFT] = linear_control_output - (angular_control_output * WHEEL_SEPARATION / 2);
  //wheel_vel[RIGHT] = linear_control_output + (angular_control_output * WHEEL_SEPARATION / 2);
  //wheel_vel[LEFT] = linear_control_output - (angular_vel * WHEEL_SEPARATION / 2 * 1000);
  //wheel_vel[RIGHT] = linear_control_output + (angular_vel * WHEEL_SEPARATION / 2 * 1000);
  
  //linearPIDCaculate(wheel_vel[LEFT]);
  left_out = leftPIDCaculate(wheel_vel[LEFT], last_velocity[LEFT] * WHEEL_RADIUS, 1000, 800);
  right_out = rightPIDCaculate(wheel_vel[RIGHT], last_velocity[RIGHT] * WHEEL_RADIUS, 1000, 800);
  
  
//  if(wheel_vel[LEFT] == 0 && wheel_vel[RIGHT] == 0)
//  {
//    //左轮停转
//    GPIO_ResetBits(GPIOB, GPIO_Pin_14);
//    GPIO_ResetBits(GPIOB, GPIO_Pin_15);
//    //右轮停转
//    GPIO_ResetBits(GPIOB, GPIO_Pin_13);
//    GPIO_ResetBits(GPIOB, GPIO_Pin_12);
//    motor_output[LEFT] = motor_output[RIGHT] = 0;
//  }
  if(wheel_vel[LEFT] > 0)
  {
    GPIO_SetBits(GPIOB, GPIO_Pin_13);
    GPIO_ResetBits(GPIOB, GPIO_Pin_12);
    motor_output[LEFT] = (uint16_t)(left_out + 50);//(/*leftPIDCaculate(wheel_vel[LEFT],last_velocity[LEFT],1000,800)*/ 0);
  }
  else
  {
    GPIO_SetBits(GPIOB, GPIO_Pin_12);
    GPIO_ResetBits(GPIOB, GPIO_Pin_13);
    motor_output[LEFT] = (uint16_t)(- left_out - 50);//(/*(PIDCaculate(wheel_vel[LEFT],last_velocity[LEFT],1000,800))*/ 0);
  }
  if(wheel_vel[RIGHT] > 0)
  {
    GPIO_SetBits(GPIOB, GPIO_Pin_14);
    GPIO_ResetBits(GPIOB, GPIO_Pin_15);
    motor_output[RIGHT] = (uint16_t)(right_out + 50); //(rightPIDCaculate(wheel_vel[RIGHT],last_velocity[RIGHT],1000,800));
  }
  else
  {
    GPIO_SetBits(GPIOB, GPIO_Pin_15);
    GPIO_ResetBits(GPIOB, GPIO_Pin_14);
    motor_output[RIGHT] = (uint16_t)(- right_out - 50);//(/*PIDCaculate(wheel_vel[RIGHT],last_velocity[RIGHT],1000,800)*/ 0);
  }
  TIM_SetCompare4(TIM2, motor_output[LEFT]); // 左轮
  TIM_SetCompare3(TIM2, motor_output[RIGHT]);  //右轮
}

//Send log message
void sendLogMsg(void)
{
  static bool log_flag = false;
  char log_msg[100];
  const char * init_log_data = "This core(v1.0.0) is compatible with TB3 burger";
  
  if(nh.connected())
  {
    if(log_flag == false)
    {
      sprintf(log_msg, "---------------------------");
      nh.loginfo(log_msg);
      
      sprintf(log_msg, "Connected to STM32F103 board!");
      nh.loginfo(log_msg);
      
      sprintf(log_msg, init_log_data);
      nh.loginfo(log_msg);
      
      sprintf(log_msg, "----------------------------");
      nh.loginfo(log_msg);
      
      log_flag = true;
    }
  }
  else
  {
    log_flag = false;
  }    
}