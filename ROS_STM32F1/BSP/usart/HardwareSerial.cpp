#include "HardwareSerial.h"
#include "hw_config.h"

HardwareSerial Serial;

HardwareSerial::HardwareSerial() {}

void HardwareSerial::begin(uint32_t baud)
{
   /*定义初始化结构体*/
  GPIO_InitTypeDef   GPIO_InitStructure;
	USART_InitTypeDef  USART_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;
	
	/* 开启GPIOA端口时钟及USART3串口时钟 */
	RCC_APB1PeriphClockCmd(USART3_CLK, ENABLE);		//使能USART3，GPIOA和端口复用时钟
  RCC_APB2PeriphClockCmd(USART3_GPIO_CLK | RCC_APB2Periph_AFIO, ENABLE);
 	
	/* 复位串口1 */
	USART_DeInit(USART3);  //复位串口1
	
	/* 配置USART3_TX（PA9）引脚GPIO状态 */
  GPIO_InitStructure.GPIO_Pin = USART3_TX_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
  GPIO_Init(USART3_TX_PORT, &GPIO_InitStructure);   

  /* 配置USART3_RX（PA10）引脚GPIO状态 */
  GPIO_InitStructure.GPIO_Pin = USART3_RX_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//上拉输入
  GPIO_Init(USART3_RX_PORT, &GPIO_InitStructure);  

  /* USART 初始化设置 */
	USART_InitStructure.USART_BaudRate = baud;  //波特率：一般设置为115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //字长：为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;  //停止位：一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;  //校验位：无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//硬件流控制:无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//工作模式：收发模式
  USART_Init(USART3, &USART_InitStructure);  //根据指定的参数初始化串口1

  /* 配置嵌套向量中断控制器NVIC */ 
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;  //配置USART3为中断源
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0; //抢占优先级2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //子优先级0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  //IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	 //根据指定的参数初始化NVIC寄存器
	
  /* 使能串口接收中断 */ 
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
  /* 串口1使能 */
  USART_Cmd(USART3, ENABLE);  //使能串口 
}

int HardwareSerial::available(void)
{
  return ((uint32_t)(SERIAL_RX_BUFFER_SIZE + _rx_buffer_head - _rx_buffer_tail)) % SERIAL_RX_BUFFER_SIZE;
  //return usb_vcp_available();
}

//int HardwareSerial::peek(void)
//{
//  if(_rx_buffer_head == _rx_buffer_tail)
//  {
//    return -1;
//  }
//  else
//  {
//    return _rx_buffer[_rx_buffer_tail];
//  }
//}

int HardwareSerial::read(void)
{
  // if the head isn't ahead of the tail, we don't have any characters
  if (_rx_buffer_head == _rx_buffer_tail) {
    return -1;
  } else {
    unsigned char c = _rx_buffer[_rx_buffer_tail];
    _rx_buffer_tail = (uint16_t)(_rx_buffer_tail + 1) % SERIAL_RX_BUFFER_SIZE;
    return c;
  }
  //return usb_vcp_read();
}

uint32_t HardwareSerial::write(uint8_t c)
{
  USART_SendData(USART3, c);
  while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
  return 1;
//  txBufferToUsbSendData(c);
//  return 1;
}

void HardwareSerial::irq()
{
  uint16_t i = (uint16_t)(_rx_buffer_head + 1) % SERIAL_RX_BUFFER_SIZE;
  uint8_t c;
  if(USART_GetFlagStatus(USART3,USART_IT_RXNE) != RESET)
  {
    c = USART_ReceiveData(USART3);
    if(i != _rx_buffer_tail)
    {
      _rx_buffer[_rx_buffer_head] = c;
      _rx_buffer_head = i;
    }
  }
}  

#ifdef __cplusplus
extern "C" {
#endif
void USART3_IRQHandler(void)
{
  Serial.irq();
}
#ifdef __cplusplus
}
#endif
  
  
