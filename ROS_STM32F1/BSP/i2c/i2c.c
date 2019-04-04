#include "../BSP/i2c/i2c.h"
#include <stdbool.h>

void SW_I2C_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
}

static void SW_I2C_Delay(void)
{
    volatile int i = 7;
    while (i)
        i--;
}

bool SW_I2C_Start(void)
{
    SDA_H;
    SCL_H;
    SW_I2C_Delay();
    if (!SDA_read)
        return false;
    SDA_L;
    SW_I2C_Delay();
    if (SDA_read)
        return false;
    SDA_L;
    SW_I2C_Delay();
    return true;
}

void SW_I2C_Stop(void)
{
    SCL_L;
    SW_I2C_Delay();
    SDA_L;
    SW_I2C_Delay();
    SCL_H;
    SW_I2C_Delay();
    SDA_H;
    SW_I2C_Delay();
}

void SW_I2C_Ack(void)
{
    SCL_L;
    SW_I2C_Delay();
    SDA_L;
    SW_I2C_Delay();
    SCL_H;
    SW_I2C_Delay();
    SCL_L;
    SW_I2C_Delay();
}

void SW_I2C_NoAck(void)
{
    SCL_L;
    SW_I2C_Delay();
    SDA_H;
    SW_I2C_Delay();
    SCL_H;
    SW_I2C_Delay();
    SCL_L;
    SW_I2C_Delay();
}

bool SW_I2C_WaitAck(void)
{
    SCL_L;
    SW_I2C_Delay();
    SDA_H;
    SW_I2C_Delay();
    SCL_H;
    SW_I2C_Delay();
    if (SDA_read) {
        SCL_L;
        return false;
    }
    SCL_L;
    return true;
}

void SW_I2C_SendByte(uint8_t byte)
{
    uint8_t i = 8;
    while (i--) {
        SCL_L;
        SW_I2C_Delay();
        if (byte & 0x80)
            SDA_H;
        else
            SDA_L;
        byte <<= 1;
        SW_I2C_Delay();
        SCL_H;
        SW_I2C_Delay();
    }
    SCL_L;
}

uint8_t SW_I2C_ReceiveByte(void)
{
    uint8_t i = 8;
    uint8_t byte = 0;

    SDA_H;
    while (i--) {
        byte <<= 1;
        SCL_L;
        SW_I2C_Delay();
        SCL_H;
        SW_I2C_Delay();
        if (SDA_read) {
            byte |= 0x01;
        }
    }
    SCL_L;
    return byte;
}

 