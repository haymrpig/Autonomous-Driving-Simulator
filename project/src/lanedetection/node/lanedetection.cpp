#include "Detectlane.h"

int main (int argc, char** argv)
{
  ros::init(argc, argv, "lanedetection_node");
  Detectlane detectlane;
  detectlane.Run();

  return EXIT_SUCCESS;
}
