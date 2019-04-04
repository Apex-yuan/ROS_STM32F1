#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#include "usart3.h"

#define BYTE0(dwTemp) (*(char *)(&dwTemp))
#define BYTE1(dwTemp) (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp) (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp) (*((char *)(&dwTemp) + 3))

extern float g_fware[8];

extern bool g_bStartBitFlag;
extern bool g_bNewLineFlag;
extern uint32_t g_nCmdCount;
extern uint8_t g_nCmdBuf[80];
extern uint8_t g_nProtocolBuf[80];


void usart3_send_char(uint8_t c);
void usart3_niming_report(uint8_t fun,uint8_t*data,uint8_t len);
void Send_Data(int16_t *Gyro,int16_t *Accel);
void vcan_sendware(uint8_t *wareaddr, uint32_t waresize);

void ProtocolCpyData(void);
void Protocol(void);

#endif 

