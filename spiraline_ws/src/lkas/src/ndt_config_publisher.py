#!/usr/bin/env python3
import rospy
from autoware_config_msgs.msg import ConfigNDT
from autoware_msgs.msg import NDTStat

if __name__ == "__main__":
    rospy.init_node('ndt_config_publisher')
    config_pub = rospy.Publisher('/config/ndt', ConfigNDT, queue_size=1)

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

    localization_success = False
    while not localization_success:
        config_pub.publish(config_msg)
        try:
            ndt_stat_msg = rospy.wait_for_message('/ndt_stat', NDTStat, timeout=1)
            localization_success = True
        except:
            pass
        
    

