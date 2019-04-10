#ifndef __USBSERIAL_H
#define __USBSERIAL_H

#include "hw_config.h"

#define UNUSED(x)  


class USBSerial  {

  public:
    USBSerial();
    void begin(uint32_t baud_count);
    void end(void);

    virtual int available(void);
    //virtual void accept(void);
    //virtual int peek(void);
    virtual int read(void);
    //virtual void flush(void);
    virtual int write(uint8_t c);
    //virtual size_t write(const uint8_t *buffer, size_t size);
    //using Print::write; // pull in write(str) from Print
    //operator bool();

    uint32_t getBaudRate(void);
    uint32_t getRxCnt(void);
    uint32_t getTxCnt(void);
    //uint32_t getRxErrCnt(void);
    //uint32_t getTxErrCnt(void);

  private:
    uint32_t baudrate;
    uint32_t rx_cnt;
    uint32_t tx_cnt;
    uint32_t rx_err_cnt;
    uint32_t tx_err_cnt;

};

extern USBSerial Serial0;


#endif /* __USB_SERIAL_H*/

