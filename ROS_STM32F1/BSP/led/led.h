#ifndef __LED_H
#define __LED_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f10x.h"

// typedef enum
// {
//   LED0,
//   LED_ALL;
// }
#define LED0                 ((uint8_t)0x01)  /*!< led 0 selected */
#define LED1                 ((uint8_t)0x02)  /*!< led 1 selected */
#define LED2                 ((uint8_t)0x04)  /*!< led 2 selected */
#define LED3                 ((uint8_t)0x08)  /*!< led 3 selected */
#define LED_ALL              ((uint8_t)0x0F)  /*!< led_all selected */

#define LED0_GPIO_CLK   RCC_APB2Periph_GPIOC
#define LED0_GPIO_PORT  GPIOC
#define LED0_GPIO_PIN   GPIO_Pin_13

void led_init(void);
void led_on(uint8_t LEDn);
void led_off(uint8_t LEDn);
void led_toggle(uint8_t LEDn);

   
#ifdef __cplusplus
 }
#endif

#endif /*__LED_H */


