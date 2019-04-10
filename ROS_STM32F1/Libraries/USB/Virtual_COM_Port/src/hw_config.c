/**
  ******************************************************************************
  * @file    hw_config.c
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


/* Includes ------------------------------------------------------------------*/

#include "usb_it.h"
#include "usb_lib.h"
#include "usb_prop.h"
#include "usb_desc.h"
#include "hw_config.h"
#include "usb_pwr.h"

/*used to implicate usb_printf() function*/
#include <string.h>	
#include <stdarg.h>		 
#include <stdio.h>	

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
static void IntToUnicode (uint32_t value , uint8_t *pbuf , uint8_t len);
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t  USB_Tx_State = 0;

uint8_t  USART_PRINTF_Buffer[USB_TX_BUFFER_SIZE];	//usb_printf发送缓冲区

uint8_t  _usb_tx_buffer[USB_TX_BUFFER_SIZE] = {0};
uint16_t _usb_tx_buffer_length = 0;
uint16_t _usb_tx_buffer_head = 0;
uint16_t _usb_tx_buffer_tail = 0;

uint8_t  _usb_rx_buffer[USB_RX_BUFFER_SIZE] = {0};
uint16_t _usb_rx_buffer_head = 0;
uint16_t _usb_rx_buffer_tail = 0;
/* Extern variables ----------------------------------------------------------*/

extern LINE_CODING linecoding; //USB 虚拟串口的配置信息

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/*******************************************************************************
* Function Name  : Set_USBClock
* Description    : Configures USB Clock input (48MHz)
* Input          : None.
* Return         : None.
*******************************************************************************/
void Set_USBClock(void)
{
  /* Select USBCLK source */
  RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_1Div5); //USBclk = PLLclk/1.5 = 48MHz
  
  /* Enable the USB clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);
}

/*******************************************************************************
* Function Name  : Enter_LowPowerMode
* Description    : Power-off system clocks and power while entering suspend mode
* Input          : None.
* Return         : None.
*******************************************************************************/
void Enter_LowPowerMode(void) //USB进入suspend模式时，MCU进入低功耗模式
{
  /* Set the device state to suspend */
  bDeviceState = SUSPENDED;
}

/*******************************************************************************
* Function Name  : Leave_LowPowerMode
* Description    : Restores system clocks and power while exiting suspend mode
* Input          : None.
* Return         : None.
*******************************************************************************/
void Leave_LowPowerMode(void)  //USB退出低功耗模式
{
  DEVICE_INFO *pInfo = &Device_Info;

  /* Set the device state to the correct state */
  if (pInfo->Current_Configuration != 0)
  {
    /* Device configured */
    bDeviceState = CONFIGURED;
  }
  else
  {
    bDeviceState = ATTACHED;
  }
  /*Enable SystemCoreClock*/
//  SystemInit();    //目前未用到低功耗模式，所以不需要重新配置系统时钟
}

/*******************************************************************************
* Function Name  : USB_Interrupts_Config
* Description    : Configures the USB interrupts
* Input          : None.
* Return         : None.
*******************************************************************************/
void USB_Interrupts_Config(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure; 
  
  /* Configure the EXTI line 18 connected internally to the USB IP */
  EXTI_ClearITPendingBit(EXTI_Line18);
  EXTI_InitStructure.EXTI_Line = EXTI_Line18; 
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  /* 2 bit for pre-emption priority, 2 bits for subpriority */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);    

  NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
    /* Enable the USB Wake-up interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USBWakeUp_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_Init(&NVIC_InitStructure);
}

/*******************************************************************************
* Function Name  : USB_Cable_Config
* Description    : Software Connection/Disconnection of USB Cable
* Input          : None.
* Return         : Status
*******************************************************************************/
void USB_Cable_Config (FunctionalState NewState)   //usb软件连接断开配置：配置1.5k上拉，此处不需要配置，硬件恒上拉
{
  if (NewState != DISABLE)
  {
    ///GPIO_ResetBits(USB_DISCONNECT, USB_DISCONNECT_PIN);
  }
  else
  {
    ///GPIO_SetBits(USB_DISCONNECT, USB_DISCONNECT_PIN);
  }
}

/*******************************************************************************
* Function Name  : USB_To_Buffer_Send_Data.
* Description    : send the received data from USB to the RX Buffer.
* Input          : data_buffer: data address.
                   Nb_bytes: number of bytes to send.
* Return         : none.
*******************************************************************************/
void USB_To_Buffer_Send_Data(uint8_t* data_buffer, uint8_t Nb_bytes)
{ 
  uint32_t i;
  uint16_t m;
  uint8_t c;
  
  for(i = 0; i < Nb_bytes; ++i)
  {
    c = data_buffer[i];
    m = (_usb_rx_buffer_head + 1) % USB_RX_BUFFER_SIZE;
    
    if(m != _usb_rx_buffer_tail)  //缓冲区未满
    {
      _usb_rx_buffer[_usb_rx_buffer_head] = c;
      _usb_rx_buffer_head = m;
    }
  }
}

/*******************************************************************************
* Function Name  : Handle_USBAsynchXfer.
* Description    : send data to USB.
* Input          : None.
* Return         : none.
*******************************************************************************/
void Handle_USBAsynchXfer (void)
{ 
  uint16_t USB_Tx_ptr;
  uint16_t USB_Tx_length;
  
  if(USB_Tx_State != 1)
  {
    if (_usb_tx_buffer_tail == USB_TX_BUFFER_SIZE)
    {
      _usb_tx_buffer_tail = 0;
    }
    
    if(_usb_tx_buffer_tail == _usb_tx_buffer_head) 
    {
      USB_Tx_State = 0; 
      return;
    }
    
    if(_usb_tx_buffer_tail > _usb_tx_buffer_head) /* rollback */
    { 
      _usb_tx_buffer_length = USB_TX_BUFFER_SIZE - _usb_tx_buffer_tail;
    }
    else 
    {
      _usb_tx_buffer_length = _usb_tx_buffer_head - _usb_tx_buffer_tail;
    }
    
    if (_usb_tx_buffer_length > VIRTUAL_COM_PORT_DATA_SIZE)
    {
      USB_Tx_ptr = _usb_tx_buffer_tail;
      USB_Tx_length = VIRTUAL_COM_PORT_DATA_SIZE;
      
      _usb_tx_buffer_tail += VIRTUAL_COM_PORT_DATA_SIZE;	
      _usb_tx_buffer_length -= VIRTUAL_COM_PORT_DATA_SIZE;	
    }
    else
    {
      USB_Tx_ptr = _usb_tx_buffer_tail;
      USB_Tx_length = _usb_tx_buffer_length;
      
      _usb_tx_buffer_tail += _usb_tx_buffer_length;
      _usb_tx_buffer_length = 0;
    }
    USB_Tx_State = 1; 
    UserToPMABufferCopy(&_usb_tx_buffer[USB_Tx_ptr], ENDP1_TXADDR, USB_Tx_length);
    SetEPTxCount(ENDP1, USB_Tx_length);
    SetEPTxValid(ENDP1); 
  }  
  
}


/*******************************************************************************
* Function Name  : TX Buffer_To_USB_Send_Data.
* Description    : send the data from TX Buffer to USB.
* Input          : data needed to send.
* Return         : none.
*******************************************************************************/
void Buffer_To_USB_Send_Data(uint8_t data)
{
  _usb_tx_buffer[_usb_tx_buffer_head] = data;
  
  _usb_tx_buffer_head++;
  
  /* To avoid buffer overflow*/
  if(_usb_tx_buffer_head == USB_TX_BUFFER_SIZE)
  {
    _usb_tx_buffer_head = 0;
  }
}

/*******************************************************************************
* Function Name  : Get_SerialNum.
* Description    : Create the serial number string descriptor.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Get_SerialNum(void)
{
  uint32_t Device_Serial0, Device_Serial1, Device_Serial2;

  Device_Serial0 = *(uint32_t*)ID1;
  Device_Serial1 = *(uint32_t*)ID2;
  Device_Serial2 = *(uint32_t*)ID3;  

  Device_Serial0 += Device_Serial2;

  if (Device_Serial0 != 0)
  {
    IntToUnicode (Device_Serial0, &Virtual_Com_Port_StringSerial[2] , 8);
    IntToUnicode (Device_Serial1, &Virtual_Com_Port_StringSerial[18], 4);
  }
}

/*******************************************************************************
* Function Name  : HexToChar.
* Description    : Convert Hex 32Bits value into char.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
static void IntToUnicode (uint32_t value , uint8_t *pbuf , uint8_t len)
{
  uint8_t idx = 0;
  
  for( idx = 0 ; idx < len ; idx ++)
  {
    if( ((value >> 28)) < 0xA )
    {
      pbuf[ 2* idx] = (value >> 28) + '0';
    }
    else
    {
      pbuf[2* idx] = (value >> 28) + 'A' - 10; 
    }
    
    value = value << 4;
    
    pbuf[ 2* idx + 1] = 0;
  }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

//USB使能连接/断线
//NewState:0,断开
//       1,允许连接	   
void USB_Connection_Config(FunctionalState NewState)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);    //使能PORTA时钟		 
	if(NewState == 1)_SetCNTR(_GetCNTR()&(~(1<<1)));//退出断电模式
	else
	{	  
		_SetCNTR(_GetCNTR()|(1<<1));  // 断电模式
		GPIOA->CRH&=0XFFF00FFF;
		GPIOA->CRH|=0X00033000;	    		  
    GPIO_ResetBits(GPIOA, GPIO_Pin_12);
	}
} 
/******************usb vcp interface function for external call*******************************/
//usb虚拟串口,printf 函数
//确保一次发送数据不超USB_USART_REC_LEN字节
void usb_printf(char* fmt,...)  
{  
	u16 i,j;
	va_list ap;
	va_start(ap,fmt);
	vsprintf((char*)USART_PRINTF_Buffer,fmt,ap);
	va_end(ap);
	i=strlen((const char*)USART_PRINTF_Buffer);//此次发送数据的长度
	for(j=0;j<i;j++)//循环发送数据
	{
		Buffer_To_USB_Send_Data(USART_PRINTF_Buffer[j]); 
	}
} 


int usb_vcp_available(void)
{
  return ((uint32_t)(USB_RX_BUFFER_SIZE + _usb_rx_buffer_head - _usb_rx_buffer_tail)) % USB_RX_BUFFER_SIZE;
}


int usb_vcp_read(void)
{
  // if the head isn't ahead of the tail, we don't have any characters
  if (_usb_rx_buffer_head == _usb_rx_buffer_tail) {
    return -1;
  } else {
    unsigned char c = _usb_rx_buffer[_usb_rx_buffer_tail];
    _usb_rx_buffer_tail = (uint16_t)(_usb_rx_buffer_tail + 1) % USB_RX_BUFFER_SIZE;
    return c;
  }
}

void  usb_vcp_write(uint8_t data)
{
  Buffer_To_USB_Send_Data(data);
}


