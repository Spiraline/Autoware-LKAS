#include "lkas_combiner.h"

namespace lkas_combiner
{
    // Constructor
    LKASCombinerNode::LKASCombinerNode()
    : private_nh_("~")
    , usingNDT(false)
    , isLKASState(false)
    , newDataReceived(false)
    {
        initForROS();
    }

    // Destructor
    LKASCombinerNode::~LKASCombinerNode()
    {
    }

    void LKASCombinerNode::initForROS()
    {
        sub_state = nh_.subscribe("/current_state", 10, &LKASCombinerNode::current_state_cb, this);
        sub_cmd_ndt = nh_.subscribe("/vehicle_cmd", 10, &LKASCombinerNode::cmd_ndt_cb, this);
        sub_cmd_lkas = nh_.subscribe("/vehicle_cmd_lkas", 10, &LKASCombinerNode::cmd_lkas_cb, this);
        sub_ndt_stat = nh_.subscribe("/ndt_stat", 10, &LKASCombinerNode::ndt_stat_cb, this);
        pub = nh_.advertise<autoware_msgs::VehicleCmd>("/vehicle_cmd_lgsvl", 1);
    }

    void LKASCombinerNode::run()
    {
        ros::Rate loop_rate(10);

        while(ros::ok()){
            ros::spinOnce();

            // printf("[Main] published : %d, usingNDT : %d\n", newDataReceived, usingNDT);

            if(newDataReceived){
                pub.publish(final_cmd_msg);
                newDataReceived = false;
            }

            loop_rate.sleep();
        }
    }

    void LKASCombinerNode::current_state_cb(const std_msgs::Int32::ConstPtr &msg){
        // 19 stands for LKAS State
        if(msg->data == 19){
            isLKASState = true;
        }
        else{
            isLKASState = false;
        }
    }

    void LKASCombinerNode::ndt_stat_cb(const autoware_msgs::NDTStat::ConstPtr& msg){
        double score = msg->score;
        int iter = msg->iteration;
        // std::cout << "iter : " << iter << ", score : " << score << std::endl;

        if(score > 30){
            usingNDT = false;
        }
        else{
            usingNDT = true;
        }
    }

    void LKASCombinerNode::cmd_ndt_cb(const autoware_msgs::VehicleCmd::ConstPtr& msg){
        // printf("[NDT] published : %d, usingNDT : %d\n", newDataReceived, usingNDT);

        if(!isLKASState && usingNDT){
            final_cmd_msg.twist_cmd.twist.linear.x = msg->twist_cmd.twist.linear.x;
            final_cmd_msg.twist_cmd.twist.angular.z = msg->twist_cmd.twist.angular.z;
            newDataReceived = true;
        }
    }

    void LKASCombinerNode::cmd_lkas_cb(const autoware_msgs::VehicleCmd::ConstPtr& msg){        
        // printf("[LKAS] published : %d, usingNDT : %d\n", newDataReceived, usingNDT);

        if(isLKASState || !usingNDT){
            final_cmd_msg.twist_cmd.twist.linear.x = msg->twist_cmd.twist.linear.x;
            final_cmd_msg.twist_cmd.twist.angular.z = msg->twist_cmd.twist.angular.z;
            newDataReceived = true;
        }
    }
}