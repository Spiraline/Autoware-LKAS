// ROS includes
#include <ros/ros.h>
#include <autoware_msgs/VehicleCmd.h>
#include <autoware_msgs/NDTStat.h>


namespace lkas_combiner
{

class LKASCombinerNode
{
public:
  LKASCombinerNode();
  ~LKASCombinerNode();

  void run();

private:
  // handle
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // publisher
  ros::Publisher pub;
  ros::Publisher stat_pub;

  // subscriber
  ros::Subscriber sub_cmd_ndt;
  ros::Subscriber sub_cmd_lkas;
  ros::Subscriber sub_ndt_stat;

  // callbacks
  void ndt_stat_cb(const autoware_msgs::NDTStat::ConstPtr &msg);
  void cmd_ndt_cb(const autoware_msgs::VehicleCmd::ConstPtr &msg);
  void cmd_lkas_cb(const autoware_msgs::VehicleCmd::ConstPtr &msg);

  // initializer
  void initForROS();
  void InitTF();

  // functions

  // variables
  bool usingNDT;
  bool newDataReceived;
  autoware_msgs::VehicleCmd final_cmd_msg;

};

}  // namespace lkas_combiner