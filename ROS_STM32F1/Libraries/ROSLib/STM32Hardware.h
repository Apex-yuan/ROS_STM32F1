#pragma once
#include "USBSerial.h"
#include "USARTSerial.h"
#include "systick.h"

#define USE_USB_SERIAL

#if defined(USE_USB_SERIAL) //use usb serial
 #define SERIAL_CLASS  USBSerial //HardwareSerial //USBSerial
 #define Serial Serial0
#elif //use usart serial
 #define SERIAL_CLASS USARTSerial
 #define Serial Serial3
#endif /* USE_USB_SERIAL*/
class STM32Hardware {
  public:
  	STM32Hardware(SERIAL_CLASS* io , long baud= 57600){
      iostream = io;
      baud_ = baud;
    }
    STM32Hardware()
    {
      iostream = &Serial;
      baud_ = 57600;
    }
    STM32Hardware(STM32Hardware& h){
	  this->iostream = iostream;
      this->baud_ = h.baud_;
    }

    void setBaud(long baud){
      this->baud_= baud;
    }

    int getBaud(){return baud_;}

    void init(){
      iostream->begin(baud_);
    }

    int read(){
     if(iostream->available()){
	  	return iostream->read();
     }else{
	    return -1;
     }    
    };

    void write(uint8_t* data, int length){
      for(int i=0; i<length; i++){
		    iostream->write(data[i]);
      }
    }

    unsigned long time(){return millis();}

  protected:
    SERIAL_CLASS* iostream;
    long baud_;
};
