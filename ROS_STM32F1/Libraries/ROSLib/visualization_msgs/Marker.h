#ifndef _ROS_visualization_msgs_Marker_h
#define _ROS_visualization_msgs_Marker_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/ColorRGBA.h"
#include "ros/duration.h"
#include "geometry_msgs/Point.h"

namespace visualization_msgs
{

  class Marker : public ros::Msg
  {
    public:
      std_msgs::Header header;
      const char* ns;
      int32_t id;
      int32_t type;
      int32_t action;
      geometry_msgs::Pose pose;
      geometry_msgs::Vector3 scale;
      std_msgs::ColorRGBA color;
      ros::Duration lifetime;
      bool frame_locked;
      uint8_t points_length;
      geometry_msgs::Point st_points;
      geometry_msgs::Point * points;
      uint8_t colors_length;
      std_msgs::ColorRGBA st_colors;
      std_msgs::ColorRGBA * colors;
      const char* text;
      const char* mesh_resource;
      bool mesh_use_embedded_materials;
      enum { ARROW = 0 };
      enum { CUBE = 1 };
      enum { SPHERE = 2 };
      enum { CYLINDER = 3 };
      enum { LINE_STRIP = 4 };
      enum { LINE_LIST = 5 };
      enum { CUBE_LIST = 6 };
      enum { SPHERE_LIST = 7 };
      enum { POINTS = 8 };
      enum { TEXT_VIEW_FACING = 9 };
      enum { MESH_RESOURCE = 10 };
      enum { TRIANGLE_LIST = 11 };
      enum { ADD = 0 };
      enum { MODIFY = 0 };
      enum { DELETE = 2 };

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      uint32_t length_ns = strlen(this->ns);
      memcpy(outbuffer + offset, &length_ns, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->ns, length_ns);
      offset += length_ns;
      union {
        int32_t real;
        uint32_t base;
      } u_id;
      u_id.real = this->id;
      *(outbuffer + offset + 0) = (u_id.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_id.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_id.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_id.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->id);
      union {
        int32_t real;
        uint32_t base;
      } u_type;
      u_type.real = this->type;
      *(outbuffer + offset + 0) = (u_type.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_type.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_type.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_type.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->type);
      union {
        int32_t real;
        uint32_t base;
      } u_action;
      u_action.real = this->action;
      *(outbuffer + offset + 0) = (u_action.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_action.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_action.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_action.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->action);
      offset += this->pose.serialize(outbuffer + offset);
      offset += this->scale.serialize(outbuffer + offset);
      offset += this->color.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->lifetime.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->lifetime.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->lifetime.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->lifetime.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->lifetime.sec);
      *(outbuffer + offset + 0) = (this->lifetime.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->lifetime.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->lifetime.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->lifetime.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->lifetime.nsec);
      union {
        bool real;
        uint8_t base;
      } u_frame_locked;
      u_frame_locked.real = this->frame_locked;
      *(outbuffer + offset + 0) = (u_frame_locked.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->frame_locked);
      *(outbuffer + offset++) = points_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < points_length; i++){
      offset += this->points[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset++) = colors_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < colors_length; i++){
      offset += this->colors[i].serialize(outbuffer + offset);
      }
      uint32_t length_text = strlen(this->text);
      memcpy(outbuffer + offset, &length_text, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->text, length_text);
      offset += length_text;
      uint32_t length_mesh_resource = strlen(this->mesh_resource);
      memcpy(outbuffer + offset, &length_mesh_resource, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->mesh_resource, length_mesh_resource);
      offset += length_mesh_resource;
      union {
        bool real;
        uint8_t base;
      } u_mesh_use_embedded_materials;
      u_mesh_use_embedded_materials.real = this->mesh_use_embedded_materials;
      *(outbuffer + offset + 0) = (u_mesh_use_embedded_materials.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->mesh_use_embedded_materials);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t length_ns;
      memcpy(&length_ns, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_ns; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_ns-1]=0;
      this->ns = (char *)(inbuffer + offset-1);
      offset += length_ns;
      union {
        int32_t real;
        uint32_t base;
      } u_id;
      u_id.base = 0;
      u_id.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_id.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_id.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_id.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->id = u_id.real;
      offset += sizeof(this->id);
      union {
        int32_t real;
        uint32_t base;
      } u_type;
      u_type.base = 0;
      u_type.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_type.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_type.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_type.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->type = u_type.real;
      offset += sizeof(this->type);
      union {
        int32_t real;
        uint32_t base;
      } u_action;
      u_action.base = 0;
      u_action.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_action.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_action.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_action.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->action = u_action.real;
      offset += sizeof(this->action);
      offset += this->pose.deserialize(inbuffer + offset);
      offset += this->scale.deserialize(inbuffer + offset);
      offset += this->color.deserialize(inbuffer + offset);
      this->lifetime.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->lifetime.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->lifetime.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->lifetime.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->lifetime.sec);
      this->lifetime.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->lifetime.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->lifetime.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->lifetime.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->lifetime.nsec);
      union {
        bool real;
        uint8_t base;
      } u_frame_locked;
      u_frame_locked.base = 0;
      u_frame_locked.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->frame_locked = u_frame_locked.real;
      offset += sizeof(this->frame_locked);
      uint8_t points_lengthT = *(inbuffer + offset++);
      if(points_lengthT > points_length)
        this->points = (geometry_msgs::Point*)realloc(this->points, points_lengthT * sizeof(geometry_msgs::Point));
      offset += 3;
      points_length = points_lengthT;
      for( uint8_t i = 0; i < points_length; i++){
      offset += this->st_points.deserialize(inbuffer + offset);
        memcpy( &(this->points[i]), &(this->st_points), sizeof(geometry_msgs::Point));
      }
      uint8_t colors_lengthT = *(inbuffer + offset++);
      if(colors_lengthT > colors_length)
        this->colors = (std_msgs::ColorRGBA*)realloc(this->colors, colors_lengthT * sizeof(std_msgs::ColorRGBA));
      offset += 3;
      colors_length = colors_lengthT;
      for( uint8_t i = 0; i < colors_length; i++){
      offset += this->st_colors.deserialize(inbuffer + offset);
        memcpy( &(this->colors[i]), &(this->st_colors), sizeof(std_msgs::ColorRGBA));
      }
      uint32_t length_text;
      memcpy(&length_text, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_text; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_text-1]=0;
      this->text = (char *)(inbuffer + offset-1);
      offset += length_text;
      uint32_t length_mesh_resource;
      memcpy(&length_mesh_resource, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_mesh_resource; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_mesh_resource-1]=0;
      this->mesh_resource = (char *)(inbuffer + offset-1);
      offset += length_mesh_resource;
      union {
        bool real;
        uint8_t base;
      } u_mesh_use_embedded_materials;
      u_mesh_use_embedded_materials.base = 0;
      u_mesh_use_embedded_materials.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->mesh_use_embedded_materials = u_mesh_use_embedded_materials.real;
      offset += sizeof(this->mesh_use_embedded_materials);
     return offset;
    }

    const char * getType(){ return "visualization_msgs/Marker"; };
    const char * getMD5(){ return "18326976df9d29249efc939e00342cde"; };

  };

}
#endif