#ifndef _ROS_sensor_msgs_Joy_h
#define _ROS_sensor_msgs_Joy_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace sensor_msgs
{

  class Joy : public ros::Msg
  {
    public:
      std_msgs::Header header;
      uint8_t axes_length;
      float st_axes;
      float * axes;
      uint8_t buttons_length;
      int32_t st_buttons;
      int32_t * buttons;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset++) = axes_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < axes_length; i++){
      union {
        float real;
        uint32_t base;
      } u_axesi;
      u_axesi.real = this->axes[i];
      *(outbuffer + offset + 0) = (u_axesi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_axesi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_axesi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_axesi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->axes[i]);
      }
      *(outbuffer + offset++) = buttons_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < buttons_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_buttonsi;
      u_buttonsi.real = this->buttons[i];
      *(outbuffer + offset + 0) = (u_buttonsi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_buttonsi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_buttonsi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_buttonsi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->buttons[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint8_t axes_lengthT = *(inbuffer + offset++);
      if(axes_lengthT > axes_length)
        this->axes = (float*)realloc(this->axes, axes_lengthT * sizeof(float));
      offset += 3;
      axes_length = axes_lengthT;
      for( uint8_t i = 0; i < axes_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_axes;
      u_st_axes.base = 0;
      u_st_axes.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_axes.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_axes.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_axes.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_axes = u_st_axes.real;
      offset += sizeof(this->st_axes);
        memcpy( &(this->axes[i]), &(this->st_axes), sizeof(float));
      }
      uint8_t buttons_lengthT = *(inbuffer + offset++);
      if(buttons_lengthT > buttons_length)
        this->buttons = (int32_t*)realloc(this->buttons, buttons_lengthT * sizeof(int32_t));
      offset += 3;
      buttons_length = buttons_lengthT;
      for( uint8_t i = 0; i < buttons_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_st_buttons;
      u_st_buttons.base = 0;
      u_st_buttons.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_buttons.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_buttons.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_buttons.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_buttons = u_st_buttons.real;
      offset += sizeof(this->st_buttons);
        memcpy( &(this->buttons[i]), &(this->st_buttons), sizeof(int32_t));
      }
     return offset;
    }

    const char * getType(){ return "sensor_msgs/Joy"; };
    const char * getMD5(){ return "5a9ea5f83505693b71e785041e67a8bb"; };

  };

}
#endif