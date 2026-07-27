#include <cstdio>

int main()
{
  std::fputs(
    "bev_processor_node was removed. Use "
    "'ros2 launch bev_processor bev_processor_auto.launch.py' or "
    "'ros2 launch bev_processor bev_processor_manual.launch.py'.\n",
    stderr);
  return 2;
}
