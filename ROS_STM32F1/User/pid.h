#ifndef __PID_H
#define __PID_H


typedef struct pid
{
    float kp;
    float ki;
    float kd;

    float error;
    float error_k1; //ǰһ��ƫ��e(k-1)
    float error_k2; //ǰǰһ��ƫ��e(k-2)
}PID_t;

void pid_set(PID_t *pid, float kp, float ki, float kd);
float pid_caculate(PID_t *pid, float current_value, float target_value);


#endif /* __PID_H */
