#pragma once
#include "USBSerial.h"
#include "HardwareSerial.h"
#include "systick.h"
#include "hw_config.h"

//use USB virtual com port
#define SERIAL_CLASS  USBSerial //HardwareSerial //USBSerial

class STM32Hardware {
  public:
  	STM32Hardware(SERIAL_CLASS* io , long baud= 57600){
      iostream = io;
      baud_ = baud;
    }
    STM32Hardware()
    {
      iostream = &Serial0;
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
