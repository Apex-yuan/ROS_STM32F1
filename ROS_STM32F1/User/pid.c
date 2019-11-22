#include "pid.h"

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


