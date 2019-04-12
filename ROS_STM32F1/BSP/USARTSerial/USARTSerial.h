#ifndef __USARTSERIAL_H
#define __USARTSERIAL_H

#include "stm32f10x.h" 

#define USARTn USART3
#define USARTn_CLK    RCC_APB1Periph_USART3
#define USARTn_GPIO_CLK RCC_APB2Periph_GPIOB

#define USARTn_TX_PORT GPIOB
#define USARTn_TX_PIN  GPIO_Pin_10
#define USARTn_RX_PORT GPIOB
#define USARTn_RX_PIN  GPIO_Pin_11

#define USARTn_IRQ(a,b)  a##b
#define USARTn_IRQHandler USARTn_IRQ(USARTn##IRQHandler)

#define USARTn_TX_BUFFER_SIZE  64//(1024*2)
#define USARTn_RX_BUFFER_SIZE  64//(1024*2)

class USARTSerial {
public:
    uint8_t           _usartn_rx_buffer[USARTn_RX_BUFFER_SIZE];
    volatile uint16_t _usartn_rx_buffer_head;
    volatile uint16_t _usartn_rx_buffer_tail;
    
    uint8_t           _usartn_tx_buffer[USARTn_TX_BUFFER_SIZE];
    volatile uint16_t _usartn_tx_buffer_head;
    volatile uint16_t _usartn_tx_buffer_tail;
    

    USARTSerial();
    ~USARTSerial(){};
    void begin(uint32_t baud);
    void end(void);
    int available(void);
    int read(void);
    bool availableForWrite(void);
    int write(uint8_t data);
    void rx_irq(void);
    void tx_irq(void);
};

extern USARTSerial Serial3;


#endif /* __USARTSERIAL_H*/
