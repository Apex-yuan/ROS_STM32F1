#ifndef __TIM_H
#define __TIM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f10x.h"


void TIM1_TIMER_Init(uint16_t arr, uint16_t psc);

  
#ifdef __cplusplus
}
#endif
  
#endif /* __TIM_H */


