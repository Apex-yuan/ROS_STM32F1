#include "USARTSerial.h"

USARTSerial Serial3;

USARTSerial::USARTSerial()
{
    //构造函数
}

void USARTSerial::begin(uint32_t baud)
{
    USART_InitTypeDef USART_InitStructure;
    GPIO_InitTypeDef  GPIO_InitStructure;
    NVIC_InitTypeDef  NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(USARTn_GPIO_CLK | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(USARTn_CLK, ENABLE);

    /*tx*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    /*rx*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    USART_DeInit(USART3);

    USART_InitStructure.USART_BaudRate = baud;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART3, &USART_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_ITConfig(USART3, USART_IT_TXE, ENABLE); // 开启发送为空中断
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE); // 开启接收为空中断

    USART_Cmd(USART3, ENABLE);
}

void USARTSerial::end(void)
{
    //none
}

int USARTSerial::available(void)
{
    return (uint32_t)(USARTn_RX_BUFFER_SIZE + _usartn_rx_buffer_head - _usartn_rx_buffer_tail) % USARTn_RX_BUFFER_SIZE;
}

int USARTSerial::read(void)
{
    if(_usartn_rx_buffer_head == _usartn_rx_buffer_tail)
    {
        return -1;
    }
    else
    {
        uint8_t c = _usartn_rx_buffer[_usartn_rx_buffer_tail];
        _usartn_rx_buffer_tail = (uint16_t)(_usartn_rx_buffer_tail + 1) % USARTn_RX_BUFFER_SIZE;
        return c;
     }
}

bool USARTSerial::availableForWrite(void)
{
    uint16_t head = (_usartn_tx_buffer_head + 1) % USARTn_TX_BUFFER_SIZE;
    uint16_t tail = _usartn_tx_buffer_tail;
    return (head != tail);
}

int USARTSerial::write(uint8_t data)
{
//    uint16_t head;
//    if(USARTSerial::availableForWrite())
//    {
//        head = _usartn_tx_buffer_head;
//        USART_ITConfig(USARTn, USART_IT_TXE, DISABLE);
//      USART_ITConfig(USARTn, USART_IT_RXNE, DISABLE);
//        _usartn_tx_buffer[head] = data;
//        _usartn_tx_buffer_head = (_usartn_tx_buffer_head + 1) % USARTn_TX_BUFFER_SIZE;
//      USART_ITConfig(USARTn, USART_IT_RXNE, ENABLE);
//      USART_ITConfig(USARTn, USART_IT_TXE, ENABLE);
//        return 1;
//    }
//    return -1;
//  USART_SendData(USART3, data);
//  while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
//  return 1;
  
  uint16_t head;
   bool avilable_write = USARTSerial::availableForWrite();
    if(avilable_write)
    {
      head = _usartn_tx_buffer_head;
      USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);
      USART_ITConfig(USART3, USART_IT_TXE, DISABLE);
      _usartn_tx_buffer[head] = data;
      _usartn_tx_buffer_head = (_usartn_tx_buffer_head + 1) % USARTn_TX_BUFFER_SIZE;
      USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
      USART_ITConfig(USART3, USART_IT_TXE, ENABLE);
    }
    return avilable_write;
}

void USARTSerial::rx_irq(void)
{
    if(USART_GetFlagStatus(USARTn, USART_IT_RXNE) != RESET)
    {
        uint16_t head = (_usartn_rx_buffer_head + 1) % USARTn_RX_BUFFER_SIZE;
        uint8_t c = USART_ReceiveData(USARTn);
        if(head != _usartn_rx_buffer_tail)
        {
            _usartn_rx_buffer[_usartn_rx_buffer_head] = c;
            _usartn_rx_buffer_head = head;
        }
    }
}

void USARTSerial::tx_irq(void)
{
    if(USART_GetITStatus(USARTn, USART_IT_TXE) != RESET)
    {
        uint16_t tail = (_usartn_tx_buffer_tail + 1) % USARTn_TX_BUFFER_SIZE;
        if(tail != _usartn_tx_buffer_head)
        {
            uint8_t data = _usartn_tx_buffer[_usartn_tx_buffer_tail];
            USART_SendData(USARTn, data);
            _usartn_tx_buffer_tail = tail;
        }
        else
        {
          USART_ITConfig(USARTn, USART_IT_TXE, DISABLE);
        }
//      uint16_t tail = _usartn_tx_buffer_tail;
//    if (_usartn_tx_buffer_head == tail)
//    {
//        USART_ITConfig(USARTn, USART_IT_TXE, DISABLE);
//    }
//    else
//    {
//        uint8_t data = _usartn_tx_buffer[_usartn_tx_buffer_tail];//TX[TX_Tail];
//        USART_SendData(USARTn, (unsigned char) data);
//        _usartn_tx_buffer_tail = (_usartn_tx_buffer_tail + 1) % (USARTn_TX_BUFFER_SIZE - 0);
//    }
    }
}

#ifdef __cplusplus
extern "C" {
#endif
  //USART3_IRQHandler
void USART3_IRQHandler(void)
//void USARTn_Event(void)
{
    Serial3.tx_irq();
    //Serial3.rx_irq();
  Serial3.rx_irq();
}

#ifdef __cplusplus
}
#endif