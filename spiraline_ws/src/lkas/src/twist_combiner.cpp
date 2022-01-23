#include "twist_combiner.h"

namespace twist_combiner
{
    // Constructor
    TwistCombinerNode::TwistCombinerNode()
    : private_nh_("~")
    , usingNDT(false)
    , isLKASState(false)
    , newDataReceived(false)
    {
        initForROS();
    }

    // Destructor
    TwistCombinerNode::~TwistCombinerNode()
    {
    }

    void TwistCombinerNode::initForROS()
    {
        sub_state = nh_.subscribe("/current_state", 10, &TwistCombinerNode::current_state_cb, this);
        sub_cmd_ndt = nh_.subscribe("/twist_cmd", 10, &TwistCombinerNode::cmd_ndt_cb, this);
        sub_cmd_lkas = nh_.subscribe("/twist_cmd_lkas", 10, &TwistCombinerNode::cmd_lkas_cb, this);
        sub_ndt_stat = nh_.subscribe("/ndt_stat", 10, &TwistCombinerNode::ndt_stat_cb, this);
        pub = nh_.advertise<geometry_msgs::TwistStamped>("/twist_cmd_merged", 1);
        nh_.param<bool>("/twist_combiner/use_lkas", _use_lkas, true);
    }

    void TwistCombinerNode::run()
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

    void TwistCombinerNode::current_state_cb(const std_msgs::Int32::ConstPtr &msg){
        // 19 stands for LKAS State
        if(msg->data == 19){
            isLKASState = true;
        }
        else{
            isLKASState = false;
        }
    }

    void TwistCombinerNode::ndt_stat_cb(const autoware_msgs::NDTStat::ConstPtr& msg){
        double p_norm = msg->p_norm;
        double score = msg->score;
        int iter = msg->iteration;
        // std::cout << "iter : " << iter << ", score : " << score << std::endl;

        if(p_norm > 0.05 && score > 3 && _use_lkas){
            usingNDT = false;
        }
        else{
            usingNDT = true;
        }
    }

    void TwistCombinerNode::cmd_ndt_cb(const geometry_msgs::TwistStamped::ConstPtr& msg){
        // printf("[NDT] published : %d, usingNDT : %d\n", newDataReceived, usingNDT);

        if(!isLKASState && usingNDT){
            final_cmd_msg.twist.linear.x = msg->twist.linear.x;
            final_cmd_msg.twist.angular.z = msg->twist.angular.z;
            newDataReceived = true;
        }
    }

    void TwistCombinerNode::cmd_lkas_cb(const geometry_msgs::TwistStamped::ConstPtr& msg){        
        // printf("[LKAS] published : %d, usingNDT : %d\n", newDataReceived, usingNDT);

        if(isLKASState || !usingNDT){
            final_cmd_msg.twist.linear.x = msg->twist.linear.x;
            final_cmd_msg.twist.angular.z = msg->twist.angular.z;
            newDataReceived = true;
        }
    }
}