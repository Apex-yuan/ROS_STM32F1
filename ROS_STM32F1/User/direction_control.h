#ifndef __DIRECTION_CONTROL_H
#define __DIRECTION_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"

#define DIRECTION_SET 0  
#define DIRECTION_CONTROL_PERIOD 10  //ms
#define DIRECTION_CONTROL_COUNT (DIRECTION_CONTROL_PERIOD / CONTROL_PERIOD) //2*5=10ms

extern float g_fDirectionControlOut;
extern float g_fDirectionControlNew, g_fDirectionControlOld;
extern uint8_t g_nDirectionControlPeriod;
extern uint16_t g_nDirectionControlCount;

void DirectionControl(void);
void DirectionControlOutput(void);

#ifdef __cplusplus
}
#endif
  
#endif /* __DIRECTION_CONTROL_H */



