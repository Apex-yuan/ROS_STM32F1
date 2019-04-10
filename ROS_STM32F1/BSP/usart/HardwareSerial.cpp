#include "HardwareSerial.h"
#include "hw_config.h"

HardwareSerial Serial;

HardwareSerial::HardwareSerial() {}

void HardwareSerial::begin(uint32_t baud)
{
   /*�����ʼ���ṹ��*/
  GPIO_InitTypeDef   GPIO_InitStructure;
	USART_InitTypeDef  USART_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;
	
	/* ����GPIOA�˿�ʱ�Ӽ�USART3����ʱ�� */
	RCC_APB1PeriphClockCmd(USART3_CLK, ENABLE);		//ʹ��USART3��GPIOA�Ͷ˿ڸ���ʱ��
  RCC_APB2PeriphClockCmd(USART3_GPIO_CLK | RCC_APB2Periph_AFIO, ENABLE);
 	
	/* ��λ����1 */
	USART_DeInit(USART3);  //��λ����1
	
	/* ����USART3_TX��PA9������GPIO״̬ */
  GPIO_InitStructure.GPIO_Pin = USART3_TX_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
  GPIO_Init(USART3_TX_PORT, &GPIO_InitStructure);   

  /* ����USART3_RX��PA10������GPIO״̬ */
  GPIO_InitStructure.GPIO_Pin = USART3_RX_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//��������
  GPIO_Init(USART3_RX_PORT, &GPIO_InitStructure);  

  /* USART ��ʼ������ */
	USART_InitStructure.USART_BaudRate = baud;  //�����ʣ�һ������Ϊ115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //�ֳ���Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;  //ֹͣλ��һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;  //У��λ������żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//Ӳ��������:��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//����ģʽ���շ�ģʽ
  USART_Init(USART3, &USART_InitStructure);  //����ָ���Ĳ�����ʼ������1

  /* ����Ƕ�������жϿ�����NVIC */ 
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;  //����USART3Ϊ�ж�Դ
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0; //��ռ���ȼ�2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //�����ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  //IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	 //����ָ���Ĳ�����ʼ��NVIC�Ĵ���
	
  /* ʹ�ܴ��ڽ����ж� */ 
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
  /* ����1ʹ�� */
  USART_Cmd(USART3, ENABLE);  //ʹ�ܴ��� 
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
  
  
