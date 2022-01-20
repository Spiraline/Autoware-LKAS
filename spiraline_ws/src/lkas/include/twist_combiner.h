// ROS includes
#include <ros/ros.h>
#include <autoware_msgs/VehicleCmd.h>
#include <autoware_msgs/NDTStat.h>
#include <std_msgs/Int32.h>


namespace twist_combiner
{

class TwistCombinerNode
{
public:
  TwistCombinerNode();
  ~TwistCombinerNode();

  void run();

private:
  // handle
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // publisher
  ros::Publisher pub;
  ros::Publisher stat_pub;

  // subscriber
  ros::Subscriber sub_state;
  ros::Subscriber sub_cmd_ndt;
  ros::Subscriber sub_cmd_lkas;
  ros::Subscriber sub_ndt_stat;

  // callbacks
  void current_state_cb(const std_msgs::Int32::ConstPtr &msg);
  void ndt_stat_cb(const autoware_msgs::NDTStat::ConstPtr &msg);
  void cmd_ndt_cb(const autoware_msgs::VehicleCmd::ConstPtr &msg);
  void cmd_lkas_cb(const autoware_msgs::VehicleCmd::ConstPtr &msg);

  // initializer
  void initForROS();
  void InitTF();

  // functions

  // variables
  bool usingNDT, isLKASState;
  bool newDataReceived;
  bool _use_lkas;
  autoware_msgs::VehicleCmd final_cmd_msg;

};

}  // namespace twist_combiner