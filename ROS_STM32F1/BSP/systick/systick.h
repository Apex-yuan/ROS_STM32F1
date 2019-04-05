#ifndef __SYSTICK_H
#define __SYSTICK_H

#ifdef __cplusplus
 extern "C" {
#endif
  
#include "stm32f10x.h"  
   
#define delay(ms) delay_ms(ms)
   
void systick_init(void);   
void delay_ms(uint32_t _millis);
void delay_us(uint32_t uillis);
uint32_t millis(void);
uint32_t micros(void);
void reset(void);
  
#ifdef __cplusplus
 }
#endif

#endif /*__SYSTICK_H */


