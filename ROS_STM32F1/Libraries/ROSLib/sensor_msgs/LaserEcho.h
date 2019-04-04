#ifndef _ROS_sensor_msgs_LaserEcho_h
#define _ROS_sensor_msgs_LaserEcho_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace sensor_msgs
{

  class LaserEcho : public ros::Msg
  {
    public:
      uint8_t echoes_length;
      float st_echoes;
      float * echoes;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset++) = echoes_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < echoes_length; i++){
      union {
        float real;
        uint32_t base;
      } u_echoesi;
      u_echoesi.real = this->echoes[i];
      *(outbuffer + offset + 0) = (u_echoesi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_echoesi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_echoesi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_echoesi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->echoes[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint8_t echoes_lengthT = *(inbuffer + offset++);
      if(echoes_lengthT > echoes_length)
        this->echoes = (float*)realloc(this->echoes, echoes_lengthT * sizeof(float));
      offset += 3;
      echoes_length = echoes_lengthT;
      for( uint8_t i = 0; i < echoes_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_echoes;
      u_st_echoes.base = 0;
      u_st_echoes.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_echoes.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_echoes.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_echoes.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_echoes = u_st_echoes.real;
      offset += sizeof(this->st_echoes);
        memcpy( &(this->echoes[i]), &(this->st_echoes), sizeof(float));
      }
     return offset;
    }

    const char * getType(){ return "sensor_msgs/LaserEcho"; };
    const char * getMD5(){ return "8bc5ae449b200fba4d552b4225586696"; };

  };

}
#endif