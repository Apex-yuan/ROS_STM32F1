#include "USBSerial.h"

USBSerial Serial0;

USBSerial::USBSerial()
{
    baudrate = 115200; 
    tx_cnt = 0;
    rx_cnt = 0;
    tx_err_cnt = 0;
    rx_err_cnt = 0;
}

void USBSerial::begin(uint32_t _baud)
{
  UNUSED(baud);
}

void USBSerial::end(void)
{

}

int USBSerial::available(void)
{
  return usb_vcp_available();
}

int USBSerial::read(void)
{
  if(USBSerial::available() == 0)
  {
    return -1;
  }
  else
  {
    rx_cnt++;
    return usb_vcp_read(); 
  }
}

int USBSerial::write(uint8_t data)
{
  usb_vcp_write(data);
  tx_cnt++;
  return 1;
}

uint32_t USBSerial::getRxCnt(void)
{
  return rx_cnt;
}

uint32_t USBSerial::getTxCnt(void)
{
  return tx_cnt;
}

uint32_t USBSerial::getBaudRate(void)
{
    return baudrate;
}