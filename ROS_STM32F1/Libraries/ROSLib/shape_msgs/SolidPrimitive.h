#ifndef _ROS_shape_msgs_SolidPrimitive_h
#define _ROS_shape_msgs_SolidPrimitive_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace shape_msgs
{

  class SolidPrimitive : public ros::Msg
  {
    public:
      uint8_t type;
      uint8_t dimensions_length;
      float st_dimensions;
      float * dimensions;
      enum { BOX = 1 };
      enum { SPHERE = 2 };
      enum { CYLINDER = 3 };
      enum { CONE = 4 };
      enum { BOX_X = 0 };
      enum { BOX_Y = 1 };
      enum { BOX_Z = 2 };
      enum { SPHERE_RADIUS = 0 };
      enum { CYLINDER_HEIGHT = 0 };
      enum { CYLINDER_RADIUS = 1 };
      enum { CONE_HEIGHT = 0 };
      enum { CONE_RADIUS = 1 };

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->type >> (8 * 0)) & 0xFF;
      offset += sizeof(this->type);
      *(outbuffer + offset++) = dimensions_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < dimensions_length; i++){
      int32_t * val_dimensionsi = (int32_t *) &(this->dimensions[i]);
      int32_t exp_dimensionsi = (((*val_dimensionsi)>>23)&255);
      if(exp_dimensionsi != 0)
        exp_dimensionsi += 1023-127;
      int32_t sig_dimensionsi = *val_dimensionsi;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_dimensionsi<<5) & 0xff;
      *(outbuffer + offset++) = (sig_dimensionsi>>3) & 0xff;
      *(outbuffer + offset++) = (sig_dimensionsi>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_dimensionsi<<4) & 0xF0) | ((sig_dimensionsi>>19)&0x0F);
      *(outbuffer + offset++) = (exp_dimensionsi>>4) & 0x7F;
      if(this->dimensions[i] < 0) *(outbuffer + offset -1) |= 0x80;
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->type =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->type);
      uint8_t dimensions_lengthT = *(inbuffer + offset++);
      if(dimensions_lengthT > dimensions_length)
        this->dimensions = (float*)realloc(this->dimensions, dimensions_lengthT * sizeof(float));
      offset += 3;
      dimensions_length = dimensions_lengthT;
      for( uint8_t i = 0; i < dimensions_length; i++){
      uint32_t * val_st_dimensions = (uint32_t*) &(this->st_dimensions);
      offset += 3;
      *val_st_dimensions = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_st_dimensions |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_st_dimensions |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_st_dimensions |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_st_dimensions = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_st_dimensions |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_st_dimensions !=0)
        *val_st_dimensions |= ((exp_st_dimensions)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->st_dimensions = -this->st_dimensions;
        memcpy( &(this->dimensions[i]), &(this->st_dimensions), sizeof(float));
      }
     return offset;
    }

    const char * getType(){ return "shape_msgs/SolidPrimitive"; };
    const char * getMD5(){ return "d8f8cbc74c5ff283fca29569ccefb45d"; };

  };

}
#endif