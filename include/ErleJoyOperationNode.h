/**
 *  This header file defines the ErleJoyOperationNode class, which is also a
 *  Node class via enhancement.
 *
 *  Version: 1.0.0
 *  Created on: 29/11/2016
 *  Modified on: 29/11/2016
 *  Author: Rafael Gomes Braga (faerugb@gmail.com)
 *  Maintainer: Rafael Gomes Braga (faerugb@gmail.com)
 */

#ifndef _ERLE_JOY_OPERATION_NODE_H_
#define _ERLE_JOY_OPERATION_NODE_H_

#include <string>
#include "Node.h"
#include "joystick.hh"
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>

#define MINRC   1100
#define BASERC  1500
#define MAXRC   1900

#define MINJOY -32767
#define MAXJOY 32767

class ErleJoyOperationNode : public Node
{
public:
  ErleJoyOperationNode(ros::NodeHandle *nh);
  virtual ~ErleJoyOperationNode();

private:
  virtual void controlLoop();

  Joystick joystick_;

  int roll_;
  int pitch_;
  int throttle_;
  int yaw_;

  // ROS objects
  ros::Publisher rc_override_pub_;
  ros::ServiceClient cl_mode_;
  ros::ServiceClient cl_arming_;

  // Member functions
  void publishRCOverride( int roll, int pitch, int throttle, int yaw );
  void setMode( std::string mode );
  void setArming( bool arming );

};

#endif // _ERLE_JOY_OPERATION_NODE_H_
