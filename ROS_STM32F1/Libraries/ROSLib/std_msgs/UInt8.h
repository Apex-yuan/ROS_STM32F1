#ifndef _ROS_std_msgs_UInt8_h
#define _ROS_std_msgs_UInt8_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace std_msgs
{

  class UInt8 : public ros::Msg
  {
    public:
      uint8_t data;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->data >> (8 * 0)) & 0xFF;
      offset += sizeof(this->data);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->data =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->data);
     return offset;
    }

    const char * getType(){ return "std_msgs/UInt8"; };
    const char * getMD5(){ return "7c8164229e7d2c17eb95e9231617fdee"; };

  };

}
#endif