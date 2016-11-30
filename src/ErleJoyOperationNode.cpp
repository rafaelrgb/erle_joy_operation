/**
 *  This source file implements the ErleJoyOperationNode class, which is also a
 *  Node class via enhancement.
 *
 *  Version: 1.0.0
 *  Created on: 29/11/2016
 *  Modified on: 29/11/2016
 *  Author: Rafael Gomes Braga (faerugb@gmail.com)
 *  Maintainer: Rafael Gomes Braga (faerugb@gmail.com)
 */

#include "ErleJoyOperationNode.h"


ErleJoyOperationNode::ErleJoyOperationNode(ros::NodeHandle *nh)
  : Node(nh, 1000),
    joystick_("/dev/input/js2")
{
    roll_ = BASERC,
    pitch_ = BASERC,
    throttle_ = BASERC,
    yaw_ = BASERC;

    rc_override_pub_ = nh->advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 10);
    cl_mode_ = nh->serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    cl_arming_ = nh->serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
}

ErleJoyOperationNode::~ErleJoyOperationNode()
{
    rc_override_pub_.shutdown();
    cl_mode_.shutdown();
    cl_arming_.shutdown();
}

void ErleJoyOperationNode::controlLoop()
{
    // Attempt to sample an event from the joystick
    JoystickEvent event;
    if (joystick_.sample(&event))
    {
      if (event.isButton())
      {
          if ( event.value != 0 )
          {
              if ( event.number == 0 )
              {
                  setMode("STABILIZE");
              }
              if ( event.number == 1 )
              {
                  setMode("ALT_HOLD");
              }
              if ( event.number == 2 )
              {
                  setMode("LOITER");
              }
              if ( event.number == 3 )
              {
                  setMode("LAND");
              }
              if ( event.number == 4 )
              {
                  setMode("RTL");
              }
              if ( event.number == 5 )
              {
                  setMode("GUIDED");
              }
              if ( event.number == 6 )
              {
                  setMode("AUTO");
              }
              if ( event.number == 7 )
              {
                  setArming(true);
              }
          }
      }
      else if (event.isAxis())
      {
          double factor = static_cast<double>(MAXRC - MINRC) / (MAXJOY - MINJOY);
          int rc_value_h = static_cast<int>( (event.value - MINJOY) * factor ) + MINRC;
          int rc_value_v = static_cast<int>( (-event.value - MINJOY) * factor ) + MINRC;

          if ( event.number == 1 )
          {
              roll_ = rc_value_h;
          }
          if ( event.number == 4 )
          {
              pitch_ = rc_value_h;
          }
          if ( event.number == 3 )
          {
              throttle_ = rc_value_v;
          }
          if ( event.number == 2 )
          {
              yaw_ = rc_value_h;
          }
          ROS_INFO("\nRoll = %d | Pitch = %d | Throttle = %d | Yaw = %d", roll_, pitch_, throttle_, yaw_);
      }
    }

    publishRCOverride( roll_, pitch_, throttle_, yaw_ );
}

void ErleJoyOperationNode::publishRCOverride( int roll, int pitch, int throttle, int yaw )
{
    mavros_msgs::OverrideRCIn msg_override;

    msg_override.channels[0] = roll;
    msg_override.channels[1] = pitch;
    msg_override.channels[2] = throttle;
    msg_override.channels[3] = yaw;
    msg_override.channels[4] = 1100;
    msg_override.channels[5] = 1100;
    msg_override.channels[6] = 1100;
    msg_override.channels[7] = 1100;

    rc_override_pub_.publish(msg_override);
}

void ErleJoyOperationNode::setMode( std::string mode )
{
    mavros_msgs::SetMode srv;
    srv.request.base_mode = 0;
    srv.request.custom_mode = mode;
    if (cl_mode_.call(srv)) {
        ROS_INFO("Send OK %d Value:", srv.response.success);
    } else {
        ROS_ERROR("Failed SetMode");
        return;
    }
}

void ErleJoyOperationNode::setArming( bool arming )
{
    mavros_msgs::CommandBool srv;
    srv.request.value = arming;
    if (cl_mode_.call(srv)) {
        ROS_INFO("Send OK %d Value:", srv.response.success);
    } else {
        ROS_ERROR("Failed SetArming");
        return;
    }
}
