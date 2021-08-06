#include "lkas_combiner.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lkas_combiner");
  lkas_combiner::LKASCombinerNode lcn;
  lcn.run();

  return 0;
}
