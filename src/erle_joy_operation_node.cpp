#include <stdlib.h>
#include "ErleJoyOperationNode.h"

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "erle_joy_operation_node");
  ErleJoyOperationNode node(new ros::NodeHandle());
  node.spin();
  return EXIT_SUCCESS;
}
