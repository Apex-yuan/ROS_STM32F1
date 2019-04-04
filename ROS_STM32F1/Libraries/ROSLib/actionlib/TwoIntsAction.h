#ifndef _ROS_actionlib_TwoIntsAction_h
#define _ROS_actionlib_TwoIntsAction_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "actionlib/TwoIntsActionGoal.h"
#include "actionlib/TwoIntsActionResult.h"
#include "actionlib/TwoIntsActionFeedback.h"

namespace actionlib
{

  class TwoIntsAction : public ros::Msg
  {
    public:
      actionlib::TwoIntsActionGoal action_goal;
      actionlib::TwoIntsActionResult action_result;
      actionlib::TwoIntsActionFeedback action_feedback;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->action_goal.serialize(outbuffer + offset);
      offset += this->action_result.serialize(outbuffer + offset);
      offset += this->action_feedback.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->action_goal.deserialize(inbuffer + offset);
      offset += this->action_result.deserialize(inbuffer + offset);
      offset += this->action_feedback.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "actionlib/TwoIntsAction"; };
    const char * getMD5(){ return "6d1aa538c4bd6183a2dfb7fcac41ee50"; };

  };

}
#endif