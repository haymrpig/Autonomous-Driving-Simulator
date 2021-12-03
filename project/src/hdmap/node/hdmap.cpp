#include "Hdmap.h"

int main (int argc, char** argv)
{
  ros::init(argc, argv, "hdmap_node");
  Hdmap hdmap;
  hdmap.Run();

  return EXIT_SUCCESS;
}
