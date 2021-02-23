
#include "dynamic_planning/dynamic_planning.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dynamic_planning_node");
  DynamicPlanning node;
  node.run();
  return 0;
}
