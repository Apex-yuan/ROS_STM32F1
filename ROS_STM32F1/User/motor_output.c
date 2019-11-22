#include "motor_output.h"
#include "pid.h"
#include "config.h"
#include "variable.h"
#include "bsp.h"


float motor_output[WHEEL_NUM];
float motor_output_old[WHEEL_NUM];
float debug_speed;
float debug_kp = 150;
float debug_ki = 150;

extern double last_velocity[WHEEL_NUM];

PID_t left_pid = {0};
PID_t right_pid = {0};

 // left motor:  gpio : PB13,PB12   PWM: PA3
 // right motor: gpio : PB14,PB15   PWM: PA2
 void motorControl(float linear_vel, float angular_vel)
 {
   float goal_vel[WHEEL_NUM], current_vel[WHEEL_NUM];
   //float motor_output[WHEEL_NUM];
  
   //������������Ŀ���ٶ�ֵ
   goal_vel[LEFT]  = linear_vel - (angular_vel * WHEEL_SEPARATION / 2);
   goal_vel[RIGHT] = linear_vel + (angular_vel * WHEEL_SEPARATION / 2);

   //�����������ӵ�ǰ�ٶ�ֵ
   current_vel[LEFT] = last_velocity[LEFT] * WHEEL_RADIUS;
   current_vel[RIGHT] = last_velocity[RIGHT] * WHEEL_RADIUS;
  
   //���PID���㵱ǰ�ĵ�����
  /* 2019/11/15
   * 1.�˴��и����ʣ�����������д����˸�ʽ��motor_output[LEFT] += pid_caculate(&left_pid, current_vel[LEFT], goal_vel[LEFT]);
   *   ��Ҫ����������Ѱ��ԭ��
   */
  pid_set(&left_pid, debug_kp, debug_ki, 0);
  pid_set(&right_pid, debug_kp, debug_ki, 0);
  motor_output[LEFT] = motor_output_old[LEFT] + pid_caculate(&left_pid, current_vel[LEFT], goal_vel[LEFT]);
  motor_output[RIGHT] = motor_output_old[RIGHT] + pid_caculate(&right_pid, current_vel[RIGHT], goal_vel[RIGHT]);
  /*��¼��������ʷֵ*/
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
  
   //�·��ٶ�Ϊ��ʱ��ȷ���������ֹͣ״̬����ֹ��������Ϊ�ƶ��������Լ�������� 
   //�ò��ּ��ϻ���ָ���ת�ķ���ʱ�����ӻ���ԭ������תһ�£�������ת��
 //  if (linear_vel == 0 && angular_vel == 0)
 //  {
 //    //����ͣת
 //    motor_setDirection(LEFT_MOTOR, STOP);
 //    //����ͣת
 //    motor_setDirection(RIGHT_MOTOR, STOP);
 //    //�������
 //    motor_output[LEFT] = motor_output[RIGHT] = 0;
 //  }

   //�������޷�
   motor_output[LEFT] = constrain(motor_output[LEFT], MIN_MOTOR_OUT, MAX_MOTOR_OUT);
   motor_output[RIGHT] = constrain(motor_output[RIGHT], MIN_MOTOR_OUT, MAX_MOTOR_OUT);

   motor_setPwm(LEFT_MOTOR, (uint16_t) fabs(motor_output[LEFT]));
   motor_setPwm(RIGHT_MOTOR, (uint16_t) fabs(motor_output[RIGHT]));
 }
