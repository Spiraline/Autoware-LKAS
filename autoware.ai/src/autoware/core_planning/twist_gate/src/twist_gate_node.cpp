/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// ROS Includes
#include <ros/ros.h>

// User defined includes
#include "twist_gate/twist_gate.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "twist_gate");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  TwistGate twist_gate(nh, private_nh);
  struct timespec start_time, end_time;

  ros::Rate loop_rate(10);

  while (ros::ok())
  {

    clock_gettime(CLOCK_MONOTONIC, &start_time);

    ros::spinOnce();

    clock_gettime(CLOCK_MONOTONIC, &end_time);
    std::string print_file_path = std::getenv("HOME");
    print_file_path.append("/Documents/tmp/twist_gate.csv");
    FILE *fp;
    fp = fopen(print_file_path.c_str(), "a");
    fprintf(fp, "%lld.%.9ld,%lld.%.9ld,%d\n",start_time.tv_sec,start_time.tv_nsec,end_time.tv_sec,end_time.tv_nsec,getpid());
    fclose(fp);

    loop_rate.sleep();
  }

  return 0;
}
