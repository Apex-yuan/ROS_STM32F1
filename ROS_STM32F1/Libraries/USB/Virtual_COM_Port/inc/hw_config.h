/**
  ******************************************************************************
  * @file    hw_config.h
  * @author  MCD Application Team
  * @version V4.0.0
  * @date    21-January-2013
  * @brief   Hardware Configuration & Setup
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HW_CONFIG_H
#define __HW_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/
  
/* Includes ------------------------------------------------------------------*/
#include "platform_config.h"
#include "usb_type.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported define -----------------------------------------------------------*/
//#define MASS_MEMORY_START     0x04002000
//#define BULK_MAX_PACKET_SIZE  0x00000040
// #define LED_ON                0xF0
// #define LED_OFF               0xFF

#define USB_TX_BUFFER_SIZE (1024*2)
#define USB_RX_BUFFER_SIZE (1024*2)

extern uint8_t  _usb_tx_buffer[USB_TX_BUFFER_SIZE];
extern uint16_t _usb_tx_buffer_length;
extern uint16_t _usb_tx_buffer_head;
extern uint16_t _usb_tx_buffer_tail;

extern uint8_t  _usb_rx_buffer[USB_RX_BUFFER_SIZE];
extern uint16_t _usb_rx_buffer_head;
extern uint16_t _usb_rx_buffer_tail;

/* Exported functions ------------------------------------------------------- */
void Set_USBClock(void);
void Enter_LowPowerMode(void);
void Leave_LowPowerMode(void);
void USB_Interrupts_Config(void);
void USB_Cable_Config (FunctionalState NewState);
void USB_To_Buffer_Send_Data(uint8_t* data_buffer, uint8_t Nb_bytes);
void Buffer_To_USB_Send_Data(uint8_t data);
void Handle_USBAsynchXfer (void);
void Get_SerialNum(void);
void USB_Connection_Config(FunctionalState NewState);

void usb_printf(char* fmt,...);
int usb_vcp_available(void);
int usb_vcp_read(void);
void  usb_vcp_write(uint8_t data);

#ifdef __cplusplus
}
#endif /* __cplusplus */

/* External variables --------------------------------------------------------*/

#endif  /*__HW_CONFIG_H*/
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
