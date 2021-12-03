#include "gui.h"

int main (int argc, char** argv)
{
  ros::init(argc, argv, "gui_node");
  Gui gui;
  gui.RunGui(argc, argv);
  return EXIT_SUCCESS;
}
