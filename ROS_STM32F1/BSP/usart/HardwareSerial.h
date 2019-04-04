#ifndef __HARDWARESERIAL_H
#define __HARDWARESERIAL_H

#include "stm32f10x.h"

#define USART3_CLK        RCC_APB1Periph_USART3
#define USART3_GPIO_CLK   RCC_APB2Periph_GPIOA

#define USART3_TX_PORT    GPIOB
#define USART3_TX_PIN     GPIO_Pin_10
#define USART3_RX_PORT    GPIOB
#define USART3_RX_PIN     GPIO_Pin_11


#define SERIAL_TX_BUFFER_SIZE  64
#define SERIAL_RX_BUFFER_SIZE  64


class HardwareSerial {
public:
  volatile uint16_t _rx_buffer_head;
  volatile uint16_t _rx_buffer_tail;
  volatile uint16_t _tx_buffer_head;
  volatile uint16_t _tx_buffer_tail;
  
  uint8_t _rx_buffer[SERIAL_RX_BUFFER_SIZE];
  uint8_t _tx_buffer[SERIAL_TX_BUFFER_SIZE];

	HardwareSerial();
	~HardwareSerial(){};
	void begin(uint32_t baud);

	int available(void);
	int read(void);
	void flush(void);
	uint32_t write(uint8_t ch);
	void print(const char *format, ...);
	void putstr(const char *str);
	void irq();

protected:
	//RingBuffer rx_buffer;
	//Serial_TypeDef Serial;
};

extern HardwareSerial Serial;

#endif /* __HARDWARESERIAL_H */

