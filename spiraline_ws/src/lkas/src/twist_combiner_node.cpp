#include "twist_combiner.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lkas_combiner");
  twist_combiner::TwistCombinerNode tcn;
  tcn.run();

  return 0;
}
