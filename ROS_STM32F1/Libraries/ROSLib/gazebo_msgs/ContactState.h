#ifndef _ROS_gazebo_msgs_ContactState_h
#define _ROS_gazebo_msgs_ContactState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/Vector3.h"

namespace gazebo_msgs
{

  class ContactState : public ros::Msg
  {
    public:
      const char* info;
      const char* collision1_name;
      const char* collision2_name;
      uint8_t wrenches_length;
      geometry_msgs::Wrench st_wrenches;
      geometry_msgs::Wrench * wrenches;
      geometry_msgs::Wrench total_wrench;
      uint8_t contact_positions_length;
      geometry_msgs::Vector3 st_contact_positions;
      geometry_msgs::Vector3 * contact_positions;
      uint8_t contact_normals_length;
      geometry_msgs::Vector3 st_contact_normals;
      geometry_msgs::Vector3 * contact_normals;
      uint8_t depths_length;
      float st_depths;
      float * depths;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_info = strlen(this->info);
      memcpy(outbuffer + offset, &length_info, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->info, length_info);
      offset += length_info;
      uint32_t length_collision1_name = strlen(this->collision1_name);
      memcpy(outbuffer + offset, &length_collision1_name, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->collision1_name, length_collision1_name);
      offset += length_collision1_name;
      uint32_t length_collision2_name = strlen(this->collision2_name);
      memcpy(outbuffer + offset, &length_collision2_name, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->collision2_name, length_collision2_name);
      offset += length_collision2_name;
      *(outbuffer + offset++) = wrenches_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < wrenches_length; i++){
      offset += this->wrenches[i].serialize(outbuffer + offset);
      }
      offset += this->total_wrench.serialize(outbuffer + offset);
      *(outbuffer + offset++) = contact_positions_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < contact_positions_length; i++){
      offset += this->contact_positions[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset++) = contact_normals_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < contact_normals_length; i++){
      offset += this->contact_normals[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset++) = depths_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < depths_length; i++){
      int32_t * val_depthsi = (int32_t *) &(this->depths[i]);
      int32_t exp_depthsi = (((*val_depthsi)>>23)&255);
      if(exp_depthsi != 0)
        exp_depthsi += 1023-127;
      int32_t sig_depthsi = *val_depthsi;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_depthsi<<5) & 0xff;
      *(outbuffer + offset++) = (sig_depthsi>>3) & 0xff;
      *(outbuffer + offset++) = (sig_depthsi>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_depthsi<<4) & 0xF0) | ((sig_depthsi>>19)&0x0F);
      *(outbuffer + offset++) = (exp_depthsi>>4) & 0x7F;
      if(this->depths[i] < 0) *(outbuffer + offset -1) |= 0x80;
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_info;
      memcpy(&length_info, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_info; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_info-1]=0;
      this->info = (char *)(inbuffer + offset-1);
      offset += length_info;
      uint32_t length_collision1_name;
      memcpy(&length_collision1_name, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_collision1_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_collision1_name-1]=0;
      this->collision1_name = (char *)(inbuffer + offset-1);
      offset += length_collision1_name;
      uint32_t length_collision2_name;
      memcpy(&length_collision2_name, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_collision2_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_collision2_name-1]=0;
      this->collision2_name = (char *)(inbuffer + offset-1);
      offset += length_collision2_name;
      uint8_t wrenches_lengthT = *(inbuffer + offset++);
      if(wrenches_lengthT > wrenches_length)
        this->wrenches = (geometry_msgs::Wrench*)realloc(this->wrenches, wrenches_lengthT * sizeof(geometry_msgs::Wrench));
      offset += 3;
      wrenches_length = wrenches_lengthT;
      for( uint8_t i = 0; i < wrenches_length; i++){
      offset += this->st_wrenches.deserialize(inbuffer + offset);
        memcpy( &(this->wrenches[i]), &(this->st_wrenches), sizeof(geometry_msgs::Wrench));
      }
      offset += this->total_wrench.deserialize(inbuffer + offset);
      uint8_t contact_positions_lengthT = *(inbuffer + offset++);
      if(contact_positions_lengthT > contact_positions_length)
        this->contact_positions = (geometry_msgs::Vector3*)realloc(this->contact_positions, contact_positions_lengthT * sizeof(geometry_msgs::Vector3));
      offset += 3;
      contact_positions_length = contact_positions_lengthT;
      for( uint8_t i = 0; i < contact_positions_length; i++){
      offset += this->st_contact_positions.deserialize(inbuffer + offset);
        memcpy( &(this->contact_positions[i]), &(this->st_contact_positions), sizeof(geometry_msgs::Vector3));
      }
      uint8_t contact_normals_lengthT = *(inbuffer + offset++);
      if(contact_normals_lengthT > contact_normals_length)
        this->contact_normals = (geometry_msgs::Vector3*)realloc(this->contact_normals, contact_normals_lengthT * sizeof(geometry_msgs::Vector3));
      offset += 3;
      contact_normals_length = contact_normals_lengthT;
      for( uint8_t i = 0; i < contact_normals_length; i++){
      offset += this->st_contact_normals.deserialize(inbuffer + offset);
        memcpy( &(this->contact_normals[i]), &(this->st_contact_normals), sizeof(geometry_msgs::Vector3));
      }
      uint8_t depths_lengthT = *(inbuffer + offset++);
      if(depths_lengthT > depths_length)
        this->depths = (float*)realloc(this->depths, depths_lengthT * sizeof(float));
      offset += 3;
      depths_length = depths_lengthT;
      for( uint8_t i = 0; i < depths_length; i++){
      uint32_t * val_st_depths = (uint32_t*) &(this->st_depths);
      offset += 3;
      *val_st_depths = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_st_depths |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_st_depths |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_st_depths |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_st_depths = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_st_depths |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_st_depths !=0)
        *val_st_depths |= ((exp_st_depths)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->st_depths = -this->st_depths;
        memcpy( &(this->depths[i]), &(this->st_depths), sizeof(float));
      }
     return offset;
    }

    const char * getType(){ return "gazebo_msgs/ContactState"; };
    const char * getMD5(){ return "48c0ffb054b8c444f870cecea1ee50d9"; };

  };

}
#endif