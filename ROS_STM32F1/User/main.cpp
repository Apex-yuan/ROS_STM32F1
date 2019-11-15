
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include "bsp.h"
#include "config.h"

#include "variable.h"
#include "protocol.h"


/* ROS NodeHandle ------------------------------------------------------------------*/
ros::NodeHandle nh;
ros::Time current_time;
uint32_t current_offset;

/* Publisher ---------------------------------------------------------------------*/
//Odometry
nav_msgs::Odometry odom;
ros::Publisher odom_pub("odom", &odom);
//Joint (Dynamixel) state of robot
sensor_msgs::JointState joint_states;
ros::Publisher joint_states_pub("joint_states", &joint_states);
//Battey state 
sensor_msgs::BatteryState battery_state_msg;
ros::Publisher battery_state_pub("battery_state", &battery_state_msg);
//IMU 
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu", &imu_msg);
//sonar
std_msgs::UInt16MultiArray sonar_msg;
ros::Publisher sonar_pub("sonar", &sonar_msg );
//rpy
std_msgs::Float32MultiArray rpy_msg;
ros::Publisher rpy_pub("rpy", &rpy_msg);

/* Tranceform Broadcaster ----------------------------------------------------------*/
geometry_msgs::TransformStamped tfs_msg;
geometry_msgs::TransformStamped odom_tf;
tf::TransformBroadcaster tf_broadcaster;

/* Suberscriber ------------------------------------------------------------------*/
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", commandVelocityCallback);
// ros::Subscriber<std_msgs::Empty > led_sub("toggle_led", ledCallback);
ros::Subscriber<std_msgs::Float32MultiArray> angle_pid_sub("set_angle_pid", anglePidCallback);
ros::Subscriber<std_msgs::Float32MultiArray> speed_pid_sub("set_speed_pid", speedPidCallback);
ros::Subscriber<std_msgs::Float32MultiArray> direction_pid_sub("set_direction_pid", directionPidCallback);

static uint32_t tTime[10];
//float goal_velocity[WHEEL_NUM] = {0.0, 0.0}; //定义在variable.c文件中
float goal_velocity_from_cmd[WHEEL_NUM] = {0.0, 0.0};

char odom_header_frame_id[30] = "/odom";
char odom_child_frame_id[30] = "/base_footprint";

char joint_state_header_frame_id[30] = "base_link";

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

//sonar data
uint16_t sonar_data[SONAR_NUM] = {1,2,3,4};

//declatation for battery
uint8_t battery_state = 0;

PID_t left_pid = {0};
PID_t right_pid = {0};

/*callback -----------------------------------------------------------*/
void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg)
{
  goal_velocity_from_cmd[LINEAR] = cmd_vel_msg.linear.x;
  goal_velocity_from_cmd[ANGULAR] = cmd_vel_msg.angular.z;

  goal_velocity_from_cmd[LINEAR] = constrain(goal_velocity_from_cmd[LINEAR], MIN_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY);
  goal_velocity_from_cmd[ANGULAR] = constrain(goal_velocity_from_cmd[ANGULAR], MIN_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);
}

//角度采用pd控制i可以为任意值
void anglePidCallback(const std_msgs::Float32MultiArray & angle_pid_msg)
{
  //pid_msg.data_length = 3;
  ros_angle_kp = angle_pid_msg.data[0];
  //ros_ki = pid_msg.data[1];
  ros_angle_kd = angle_pid_msg.data[2];
}

//速度采用pi控制d可以为任意值
void speedPidCallback(const std_msgs::Float32MultiArray & speed_pid_msg)
{
  ros_speed_kp = speed_pid_msg.data[0];
  ros_speed_ki = speed_pid_msg.data[1];
}

//方向采用pi控制d可以为任意值
void directionPidCallback(const std_msgs::Float32MultiArray & direction_pid_msg)
{
  ros_direction_kp = direction_pid_msg.data[0];
  ros_direction_ki = direction_pid_msg.data[1];
}

// void ledCallback(const std_msgs::Empty& led_msg)
// {
//   //GPIO_WriteBit(GPIOC, GPIO_Pin_13, (BitAction)(1 - GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_13)));
// }

/*******************************************************************************

* Wait for Serial Link

*******************************************************************************/

void waitForSerialLink(bool isConnected)
{
  static bool wait_flag = false; 

  if (isConnected)
  {
    if (wait_flag == false)
    {      
      delay_ms(20);
      wait_flag = true;
    }
  }
  else
  {
    wait_flag = false;
  }
}


int kp = 5;
int16_t debug_l,debug_r;
int main()
{
  
  //init hardware periph
  bsp_init();
  
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  
  nh.subscribe(cmd_vel_sub);
  //nh.subscribe(led_sub);
  nh.subscribe(angle_pid_sub);
  nh.subscribe(speed_pid_sub);
  nh.subscribe(direction_pid_sub);

  nh.advertise(imu_pub);
  nh.advertise(odom_pub);
  nh.advertise(joint_states_pub);
  nh.advertise(sonar_pub);
  nh.advertise(rpy_pub);
  nh.advertise(battery_state_pub);

  tf_broadcaster.init(nh);
  
  /*订阅消息初始化*/
  
  /*发布信息初始化*/
  initSonar();
  initRpy();
  initOdom();
  initJointStates();
  
  prev_update_time = millis();
//   while(1)
//   {
//     debug_l = (int16_t)TIM_GetCounter(TIM3);
//     debug_r = -(int16_t)TIM_GetCounter(TIM4);
//     delay_ms(100);
//   }
   
 while(1)
 {
   uint32_t t = millis();
   updateTime();
   
   if((t - tTime[0]) >= (1000 / CMD_VEL_PUBLISH_FREQUENCY))
   {  
     //nh.getParam("pid_p",(char **)kp);
     //nh.loginfo("pid_p=%d");
     //usb_printf("pid_p=%d",kp);
     //ROS_INFO("dddd");
     updateGoalVelocity();
     motorControl(goal_velocity[LINEAR], goal_velocity[ANGULAR]);
     tTime[0] = t;
   }  
   if((t - tTime[2]) >= (1000 / DRIVE_INFORMATION_PUBLISH_FREQUENCY))
   {
     publishBatteryStateMsg();
     publishDriveInformation();
     tTime[2] = t;
   }    
  if((t - tTime[3]) >= (1000 / IMU_PUBLISH_FREQUENCY))
  {
    publishImuMsg();
    tTime[3] = t;
  }
  if((t - tTime[4]) > 1000 / 10)
  {
    publishRpyMsg();
    publishSonarMsg();
    tTime[4] = t;
  }
  if((t - tTime[5]) >= 1000 / LED_BRINK_FERQUENCE)
   {
     if(nh.connected())
     {
       led_toggle(LED0);
     }
     tTime[5] = t;
   }

   // Send log message after ROS connection
   sendLogMsg();
       
   //
   //MPU_DMP_ReadData(&imu_data);
   
   //处理蓝牙消息
   protocol_process();
   vcan_sendware((uint8_t *)g_fware,sizeof(g_fware)); 
   
   nh.spinOnce();
   //delay_ms(20);  //
  waitForSerialLink(nh.connected());
 }
}


void initSonar(void)
{
  sonar_msg.layout.data_offset = 0;
  sonar_msg.data_length = SONAR_NUM;
  for(uint8_t i = 0; i < SONAR_NUM; ++i)
  {
    sonar_msg.data[i] = 0;
  }
}
void initRpy(void)
{
  rpy_msg.layout.data_offset = 0;
  rpy_msg.data_length = 3;
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

void publishSonarMsg(void)
{
  sonar_data[millis() % SONAR_NUM] = millis() % 415; //暂时用来模拟超声波数据的变化
  sonar_msg.data = sonar_data;
  sonar_pub.publish(&sonar_msg);
}
float rpy_data[3];
//publish msgs(rpy 欧拉角)
void publishRpyMsg(void)
{
  float yaw,delta_yaw;
  static float last_yaw = 0;
  //static float yaw_intergral = 0;
  
  // static uint32_t last_time;
  // uint32_t time = millis();
  // uint32_t delta_time = time - last_time;
  // last_time = time;

  //通过该方式减弱yaw方向的温漂
  yaw = imu_data.rpy[2];
  delta_yaw = yaw - last_yaw - 0.000010;
  last_yaw = yaw;

  rpy_data[0] = imu_data.rpy[0];
  rpy_data[1] = imu_data.rpy[1];
  rpy_data[2] += delta_yaw;
  
  rpy_msg.data = rpy_data;
  //rpy_msg.data[1] = imu_data.rpy[1];
  //rpy_msg.data[2] = imu_data.rpy[2];//yaw_intergral;
  rpy_pub.publish(&rpy_msg);
}

//publish msgs(battery_state)
void publishBatteryStateMsg(void)
{
  battery_state_msg.header.stamp = rosNow();
  battery_state_msg.design_capacity = 1.8f; //Ah
  battery_state_msg.voltage = 10.8; //此处为假数据
  battery_state_msg.percentage = (float)(battery_state_msg.voltage / 11.1f);
  if(battery_state == 0)
  {
    battery_state_msg.present = false;
  }
  else
  {
    battery_state_msg.present = true;
  }
  battery_state_pub.publish(&battery_state_msg);
}

//Publish msgs (IMU data:angular velocity, linear acceleration, orientation)
void publishImuMsg(void)
{
  imu_msg.header.stamp  = rosNow();
  imu_msg.header.frame_id = "imu_link";
  
  imu_msg.angular_velocity.x = imu_data.gyro[0];
  imu_msg.angular_velocity.y = imu_data.gyro[1];
  imu_msg.angular_velocity.z = imu_data.gyro[2];
  imu_msg.angular_velocity_covariance[0] = 0.02;
  imu_msg.angular_velocity_covariance[1] = 0;
  imu_msg.angular_velocity_covariance[2] = 0;
  imu_msg.angular_velocity_covariance[3] = 0;
  imu_msg.angular_velocity_covariance[4] = 0.02;
  imu_msg.angular_velocity_covariance[5] = 0;
  imu_msg.angular_velocity_covariance[6] = 0;
  imu_msg.angular_velocity_covariance[7] = 0;
  imu_msg.angular_velocity_covariance[8] = 0.02;
  
  imu_msg.linear_acceleration.x = imu_data.accel[0];
  imu_msg.linear_acceleration.y = imu_data.accel[1];
  imu_msg.linear_acceleration.z = imu_data.accel[2];
  imu_msg.linear_acceleration_covariance[0] = 0.04;
  imu_msg.linear_acceleration_covariance[1] = 0;
  imu_msg.linear_acceleration_covariance[2] = 0;
  imu_msg.linear_acceleration_covariance[3] = 0;
  imu_msg.linear_acceleration_covariance[4] = 0.04;
  imu_msg.linear_acceleration_covariance[5] = 0;
  imu_msg.linear_acceleration_covariance[6] = 0;
  imu_msg.linear_acceleration_covariance[7] = 0;
  imu_msg.linear_acceleration_covariance[8] = 0.04;
  
  imu_msg.orientation.w = imu_data.quat[0];
  imu_msg.orientation.x = imu_data.quat[1];
  imu_msg.orientation.y = imu_data.quat[2];
  imu_msg.orientation.z = imu_data.quat[3];
  
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
}

int32_t encoder_l, encoder_r;
void publishDriveInformation(void)
{
  unsigned long time_now = millis();
  unsigned long step_time = time_now - prev_update_time;
  //int16_t encoder_l, encoder_r;
  
  prev_update_time = time_now;
  ros::Time stamp_now = rosNow(); //rosNow();
  
  // encoder_l = left_encoder_count;  //(int16_t)TIM_GetCounter(TIM3);
  // encoder_r = right_encoder_count; //- (int16_t)TIM_GetCounter(TIM4);
  encoder_l = (int16_t)TIM_GetCounter(TIM3);
  encoder_r = -(int16_t)TIM_GetCounter(TIM4);

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
  
 if (init_encoder)
 {
   for (int index = 0; index < WHEEL_NUM; index++)
   {
     last_diff_tick[index] = 0.0;
     last_tick[index]      = 0.0;
     last_rad[index]       = 0.0;

     last_velocity[index]  = 0.0;
   }  

   last_tick[LEFT] = left_tick;
   last_tick[RIGHT] = right_tick;

   init_encoder = false;
   return;
 }

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
//  theta = WHEEL_RADIUS * (wheel_r - wheel_l) / WHEEL_SEPARATION;
//  delta_theta = theta;
  
  //通过IMU数据计算偏转角raw
  theta = imu_data.rpy[2];//atan2f((quat[1] * quat[2] + quat[0] * quat[3]), 0.5f - quat[2] * quat[2] - quat[3] * quat[3]);
  delta_theta = theta - last_theta - 0.000003;
  
  //compute odometric pose
  odom_pose[0] += 0.98 * delta_s * cos(odom_pose[2] + (delta_theta / 2.0));
  odom_pose[1] += 1 * delta_s * sin(odom_pose[2] + (delta_theta / 2.0));
//  odom_pose[2] += theta;  //odom
  odom_pose[2] += delta_theta;  //imu
  
  g_fware[5] = imu_data.gyro[2];
    g_fware[6] = odom_pose[2];
    
  //compute odometric instantaneouse velocity
  v = delta_s / step_time;
  //w = theta / step_time;  //odom
  w = delta_theta / step_time; //imu
  
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
  odom.header.frame_id = odom_header_frame_id;
  odom.child_frame_id = odom_child_frame_id;
  
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
  odom_tf.child_frame_id = odom.child_frame_id;
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

//设定pid参数
void pid_set(PID_t *pid, float kp, float ki, float kd)
{
  (*pid).kp = kp;
  (*pid).ki = ki;
  (*pid).kd = kd;
}

//增量pid计算公式
float pid_caculate(PID_t *pid, float current_value, float target_value)
{
    float increment;
    (*pid).error = target_value - current_value;
    increment = (*pid).kp * ((*pid).error - (*pid).error_k1)                           //比例项
                + (*pid).ki * (*pid).error                                             //积分项
                + (*pid).kd * ((*pid).error - 2 * (*pid).error_k1 + (*pid).error_k2 ); //微分项
    (*pid).error_k2 = (*pid).error_k1;
    (*pid).error_k1 = (*pid).error;
    
    return increment;
}

float motor_output[WHEEL_NUM];
float motor_output_old[WHEEL_NUM];
float debug_speed;
float debug_kp = 150;
float debug_ki = 150;

 // left motor:  gpio : PB13,PB12   PWM: PA3
 // right motor: gpio : PB14,PB15   PWM: PA2
 void motorControl(float linear_vel, float angular_vel)
 {
   float goal_vel[WHEEL_NUM], current_vel[WHEEL_NUM];
   //float motor_output[WHEEL_NUM];
  
   //计算左右轮子目标速度值
   goal_vel[LEFT]  = linear_vel - (angular_vel * WHEEL_SEPARATION / 2);
   goal_vel[RIGHT] = linear_vel + (angular_vel * WHEEL_SEPARATION / 2);

   //计算左右轮子当前速度值
   current_vel[LEFT] = last_velocity[LEFT] * WHEEL_RADIUS;
   current_vel[RIGHT] = last_velocity[RIGHT] * WHEEL_RADIUS;
  
   //结合PID计算当前的电机输出
  /* 2019/11/15
   * 1.此处有个疑问：电机输出不能写成如此格式：motor_output[LEFT] += pid_caculate(&left_pid, current_vel[LEFT], goal_vel[LEFT]);
   *   需要后续测试中寻找原因！
   */
  pid_set(&left_pid, debug_kp, debug_ki, 0);
  pid_set(&right_pid, debug_kp, debug_ki, 0);
  motor_output[LEFT] = motor_output_old[LEFT] + pid_caculate(&left_pid, current_vel[LEFT], goal_vel[LEFT]);
  motor_output[RIGHT] = motor_output_old[RIGHT] + pid_caculate(&right_pid, current_vel[RIGHT], goal_vel[RIGHT]);
  /*记录电机输出历史值*/
  motor_output_old[LEFT] = motor_output[LEFT];
  motor_output_old[RIGHT] = motor_output[RIGHT];
   
   g_fware[0] = last_velocity[LEFT];
   g_fware[1] = last_velocity[RIGHT];
   g_fware[2] = current_vel[LEFT] * 100;
   g_fware[3] = current_vel[RIGHT] * 100;
   g_fware[4] = motor_output[LEFT];
   g_fware[5] = motor_output[RIGHT];
  
   if(motor_output[LEFT] > 0)
   {
     motor_setDirection(LEFT_MOTOR, FRONT);
     motor_output[LEFT] = motor_output[LEFT] + LEFT_MOTOR_OUT_DEAD_ZONE; // +
   }
   else
   {
     motor_setDirection(LEFT_MOTOR, BACK);
     motor_output[LEFT] = motor_output[LEFT] - LEFT_MOTOR_OUT_DEAD_ZONE; // +
   }

   if(motor_output[RIGHT] > 0)
   {
     motor_setDirection(RIGHT_MOTOR, FRONT);
     motor_output[RIGHT] = motor_output[RIGHT] + RIGHT_MOTOR_OUT_DEAD_ZONE; // +
   }
   else
   {
     motor_setDirection(RIGHT_MOTOR, BACK);
     motor_output[RIGHT] =  motor_output[RIGHT] - RIGHT_MOTOR_OUT_DEAD_ZONE; // +
   }
  
   //下发速度为零时，确保电机处于停止状态，防止机器人因为推动，出现自己动的情况 
   //该部分加上会出现更换转的方向时，轮子会沿原来方向转一下，再往回转。
 //  if (linear_vel == 0 && angular_vel == 0)
 //  {
 //    //左轮停转
 //    motor_setDirection(LEFT_MOTOR, STOP);
 //    //右轮停转
 //    motor_setDirection(RIGHT_MOTOR, STOP);
 //    //输出清零
 //    motor_output[LEFT] = motor_output[RIGHT] = 0;
 //  }

   //电机输出限幅
   motor_output[LEFT] = constrain(motor_output[LEFT], MIN_MOTOR_OUT, MAX_MOTOR_OUT);
   motor_output[RIGHT] = constrain(motor_output[RIGHT], MIN_MOTOR_OUT, MAX_MOTOR_OUT);

   motor_setPwm(LEFT_MOTOR, (uint16_t) abs(motor_output[LEFT]));
   motor_setPwm(RIGHT_MOTOR, (uint16_t) abs(motor_output[RIGHT]));
 }

//Send log message
void sendLogMsg(void)
{
  static bool log_flag = false;
  char log_msg[100];
  const char * init_log_data = "This core(v2.0.0) is compatible with balance car";
  
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

/*******************************************************************************
* Update the base time for interpolation
*******************************************************************************/
void updateTime() 
{
  current_offset = millis(); 
  current_time = nh.now();
}

/*******************************************************************************
* ros::Time::now() implementation
*******************************************************************************/
ros::Time rosNow()
{
  return nh.now(); 
}

/*******************************************************************************
* Time Interpolation function
*******************************************************************************/
 ros::Time addMicros(ros::Time & t, uint32_t _micros)
 {
   uint32_t sec, nsec;

   sec  = _micros / 1000 + t.sec;
   nsec = _micros % 1000000000 + t.nsec;

   return ros::Time(sec, nsec);
 }

#ifdef __cplusplus
extern "C" {
#endif
  
/*
 * 2019/11/15
 * 暂时保留该中断服务函数，目前该中断服务函数主要负责mpu数据的更新处理，后续如果有其他传感器数据也可以放到此处更新处理。
 * 1. mpu数据更新周期   5ms
 */

/*控制周期*/
#define CONTROL_PERIOD 5 //5ms
/*平衡控制变量*/
uint8_t g_n1MsEventCount = 0;
  
void TIM1_UP_IRQHandler(void)
{
  if(TIM_GetFlagStatus(TIM1, TIM_IT_Update) != RESET)
  {
    TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
    
    //中断服务程序：
    g_n1MsEventCount ++;

    if(g_n1MsEventCount >= CONTROL_PERIOD)
    {
      g_n1MsEventCount = 0;
      //vcan_sendware((uint8_t *)g_fware,sizeof(g_fware));  //该函数放在这会使程序卡死在delay_ms函数中，原因不明 2019/7/1 17:41 
    }
    else if(g_n1MsEventCount == 1)
    {
      //更新IMU数据
      MPU_DMP_ReadData(&imu_data);  //该函数放在while循环中，小车电机就会失控，尚不清楚原因。
      g_fware[7] = imu_data.rpy[2];
    }
    else if(g_n1MsEventCount == 2)
    {
    }
    else if (g_n1MsEventCount == 3)
    {
    }
    else if(g_n1MsEventCount == 4)
    {
    }
  }
}
#ifdef __cplusplus
 }
#endif
 


 

