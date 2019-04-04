#ifndef __DIRECTION_CONTROL_H
#define __DIRECTION_CONTROL_H

#include "stdint.h"

#define DIRECTION_CONTROL_PERIOD 10



extern float g_fDirectionControlOut;
extern float g_fDirectionControlNew, g_fDirectionControlOld;
extern uint8_t g_nDirectionControlPeriod;
extern uint16_t g_nDirectionControlCount;

extern float g_fDirectionSet;

void DirectionControl(void);
void DirectionControlOutput(void);

#endif /* __DIRECTION_CONTROL_H */



