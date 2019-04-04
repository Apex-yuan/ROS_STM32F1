#include "direction_control.h"

float g_fDirectionControlOut = 0;
float g_fDirectionControlNew, g_fDirectionControlOld;
uint8_t g_nDirectionControlPeriod;
uint16_t g_nDirectionControlCount;

float g_fDirectionSet;

void DirectionControl()
{
//  float fValue; 
  g_fDirectionControlNew = g_fDirectionSet;
  
  g_fDirectionControlOld = g_fDirectionControlNew;
}


void DirectionControlOutput(void)
{
  float fValue;
  
  fValue = g_fDirectionControlNew - g_fDirectionControlOld;
  g_fDirectionControlOut = fValue * (g_nDirectionControlPeriod + 1) / 
                           DIRECTION_CONTROL_PERIOD + g_fDirectionControlOld; 
}


