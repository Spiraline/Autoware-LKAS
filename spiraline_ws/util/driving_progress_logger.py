import rospy
import argparse
from autoware_msgs.msg import LaneArray, NDTStat
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray
import math
import time
import csv
from os import getenv, makedirs

def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

def dis(wp1, wp2):
    return math.sqrt((wp1[0] - wp2[0]) * (wp1[0] - wp2[0]) + (wp1[1] - wp2[1]) * (wp1[1] - wp2[1]))

def find_closest_point(map_wp_list, wp, yaw_deg):
    min_distance = 500
    min_wp = [0, 0]

    for map_wp in map_wp_list:
        if dis(map_wp, wp) < min_distance:
            min_distance = dis(map_wp, wp)
            min_wp = map_wp

    if 45 <= yaw_deg and yaw_deg < 135:
        if wp[0] < min_wp[0]:
            min_distance *= -1
    elif 135 <= yaw_deg and yaw_deg < 225:
        if wp[1] < min_wp[1]:
            min_distance *= -1
    elif 225 <= yaw_deg and yaw_deg < 315:
        if wp[0] > min_wp[0]:
            min_distance *= -1
    elif wp[1] > min_wp[1]:
        min_distance *= 1
    
    return min_wp, min_distance

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='')
    parser.add_argument('--output_file', '-o', type=str, required=True, help='output log file name')
    args = parser.parse_args()

    if args.output_file.split('.')[-1] != 'csv':
        print('Output file should be csv file!')
        exit(1)

    rospy.init_node('driving_progress_logger')

    map_wp_list = []

    gnss_x = 0
    gnss_y = 0

    lane_msg = rospy.wait_for_message('/lane_waypoints_array', LaneArray, timeout=None)

    for wp in lane_msg.lanes[0].waypoints:
        map_wp_list.append([wp.pose.pose.position.x, wp.pose.pose.position.y])

    ndt_log_dir = getenv("HOME") + "/spiraline_ws/log/ndt"
    makedirs(ndt_log_dir, exist_ok=True)

    with open(ndt_log_dir + "/" + args.output_file, "w") as f:
        wr = csv.writer(f)
        wr.writerow(['ts', 'state', 'center_offset', 'res_t'])
        while not rospy.is_shutdown():
            gnss_msg = rospy.wait_for_message('/gnss_pose', PoseStamped, timeout=None)
            state_msg = rospy.wait_for_message('/behavior_state', MarkerArray, timeout=None)
            ndt_msg = rospy.wait_for_message('/ndt_stat', NDTStat, timeout=None)
            gnss_x = round(gnss_msg.pose.position.x, 3)
            gnss_y = round(gnss_msg.pose.position.y, 3)
            ori_x = gnss_msg.pose.orientation.x
            ori_y = gnss_msg.pose.orientation.y
            ori_z = gnss_msg.pose.orientation.z
            ori_w = gnss_msg.pose.orientation.w
            r, p, y = euler_from_quaternion(ori_x, ori_y, ori_z, ori_w)

            yaw_deg = (y * 180 / math.pi + 1800) % 360
            
            min_wp, min_dis = find_closest_point(map_wp_list, [gnss_x, gnss_y], yaw_deg)

            if state_msg.markers[0].text == '(0)LKAS':
                state_text = 'Backup'
            else:
                state_text = 'Normal'

            wr.writerow([time.clock_gettime(time.CLOCK_MONOTONIC), state_text, str(min_dis), str(ndt_msg.exe_time)])