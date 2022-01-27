#!/usr/bin/env python3
from time import sleep
import rospy
from autoware_config_msgs.msg import ConfigNDT
from autoware_msgs.msg import NDTStat
from geometry_msgs.msg import PoseWithCovarianceStamped

if __name__ == "__main__":
    rospy.init_node('ndt_config_publisher')
    config_pub = rospy.Publisher('/config/ndt', ConfigNDT, queue_size=1)
    initialpose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)

    config_msg = ConfigNDT()

    config_msg.x = rospy.get_param('init_x', default=0.0)
    config_msg.y = rospy.get_param('init_y', default=0.0)
    config_msg.z = rospy.get_param('init_z', default=0.0)
    config_msg.roll = rospy.get_param('init_roll', default=0.0)
    config_msg.pitch = rospy.get_param('init_pitch', default=0.0)
    config_msg.yaw = rospy.get_param('init_yaw', default=0.0)
    config_msg.init_pos_gnss = rospy.get_param('init_pos_gnss', default=0)
    config_msg.use_predict_pose = rospy.get_param('use_predict_pose', default=1)
    config_msg.error_threshold = rospy.get_param('init_pos_gnss', default=0.01)
    config_msg.resolution = rospy.get_param('resolution', default=1.0)
    config_msg.step_size = rospy.get_param('step_size', default=0.5)
    config_msg.trans_epsilon = rospy.get_param('trans_epsilon', default=0.01)
    config_msg.max_iterations = rospy.get_param('max_iterations', default=30)

    initialpose_msg = PoseWithCovarianceStamped()
    initialpose_msg.header.frame_id = "map"
    initialpose_msg.pose.pose.position.x = config_msg.x
    initialpose_msg.pose.pose.position.y = config_msg.y
    initialpose_msg.pose.pose.position.z = config_msg.z
    initialpose_msg.pose.pose.orientation.x = 0
    initialpose_msg.pose.pose.orientation.y = 0
    initialpose_msg.pose.pose.orientation.z = 0.7
    initialpose_msg.pose.pose.orientation.w = 0.7

    config_pub.publish(config_msg)

    localization_success_cnt = 0
    while localization_success_cnt < 5:
        try:
            ndt_stat_msg_1 = rospy.wait_for_message('/ndt_stat', NDTStat, timeout=1)
            if ndt_stat_msg_1.score < 1.0:
                sleep(0.5)
                ndt_stat_msg_2 = rospy.wait_for_message('/ndt_stat', NDTStat, timeout=1)
                if ndt_stat_msg_2.score < 1.0:
                    localization_success_cnt += 1
                else:
                    localization_success_cnt = 0
            sleep(0.5)
        except:
            pass

        config_pub.publish(config_msg)
        initialpose_pub.publish(initialpose_msg)
        
    

