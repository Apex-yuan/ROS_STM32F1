#include "pid.h"

//�趨pid����
void pid_set(PID_t *pid, float kp, float ki, float kd)
{
  (*pid).kp = kp;
  (*pid).ki = ki;
  (*pid).kd = kd;
}

//����pid���㹫ʽ
float pid_caculate(PID_t *pid, float current_value, float target_value)
{
    float increment;
    (*pid).error = target_value - current_value;
    increment = (*pid).kp * ((*pid).error - (*pid).error_k1)                           //������
                + (*pid).ki * (*pid).error                                             //������
                + (*pid).kd * ((*pid).error - 2 * (*pid).error_k1 + (*pid).error_k2 ); //΢����
    (*pid).error_k2 = (*pid).error_k1;
    (*pid).error_k1 = (*pid).error;
    
    return increment;
}


