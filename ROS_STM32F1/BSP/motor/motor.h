#ifndef __MOTOR_H
#define __MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f10x.h"

#define MOTOR_TIM  TIM2
#define MOTOR_TIM_CLK  RCC_APB1Periph_TIM2


#define MOTOR_PWM_GPIO_PORT GPIOA
#define MOTOR_PWM_GPIO_CLK RCC_APB2Periph_GPIOA
#define L_MOTOR_PWM_PIN  GPIO_Pin_3
#define L_MOTOR_TIM_CHANNEL  TIM_Channel_3
#define R_MOTOR_PWM_PIN  GPIO_Pin_2
#define R_MOTOR_TIM_CHANNEL  TIM_Channel_4

#define MOTOR_IN_GPIO_PORT GPIOB
#define MOTOR_IN_GPIO_CLK RCC_APB2Periph_GPIOB
#define L_MOTOR_IN1_PIN  GPIO_Pin_13
#define L_MOTOR_IN2_PIN  GPIO_Pin_12
#define R_MOTOR_IN1_PIN  GPIO_Pin_14
#define R_MOTOR_IN2_PIN  GPIO_Pin_15


typedef enum 
{
  LEFT_MOTOR = 0,
  RIGHT_MOTOR
}MotorChoice;

typedef enum direction_motor
{
  FRONT = 0,
  BACK    , 
  STOP
}MotorDirection;

void motor_init(void);
// void TIM2_PWM_Init(uint16_t arr, uint16_t psc);
void motor_setDirection(MotorChoice choice, MotorDirection direction);
void motor_setPwm(MotorChoice choice, uint16_t pwm);

#ifdef __cplusplus
 }
#endif

#endif /*__MOTOR_H*/

