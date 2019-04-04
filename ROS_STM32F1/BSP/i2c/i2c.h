#ifndef __SW_I2C_H
#define __SW_I2C_H

//#ifdef __cpluslus
//extern "C" {
//#endif

#include "stm32f10x.h"
#include <stdbool.h>



#define SCL_H         GPIOB->BSRR = GPIO_Pin_8
#define SCL_L         GPIOB->BRR  = GPIO_Pin_8 

#define SDA_H         GPIOB->BSRR = GPIO_Pin_9
#define SDA_L         GPIOB->BRR  = GPIO_Pin_9 

#define SCL_read      GPIOB->IDR  & GPIO_Pin_8
#define SDA_read      GPIOB->IDR  & GPIO_Pin_9 

void SW_I2C_Init(void);
bool SW_I2C_Start(void);
void SW_I2C_Stop(void);
void SW_I2C_Ack(void);
void SW_I2C_NoAck(void);
bool SW_I2C_WaitAck(void);
void SW_I2C_SendByte(uint8_t byte);
uint8_t SW_I2C_ReceiveByte(void);

//#ifdef __cplusplus
// }
//#endif
  
#endif /* __SW_I2C_H */

