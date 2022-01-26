import rospy
from autoware_msgs.msg import NDTStat
from os import getenv, makedirs

def ndt_stat_cb(data, path):
    with open(path, 'a') as f:
        f.write('%d,%f\n' % (data.iteration, data.exe_time))

if __name__ == "__main__":
    res_t_directory = getenv("HOME") + "/spiraline_ws/log/ndt"
    makedirs(res_t_directory, exist_ok=True)
    lc_file = res_t_directory + "/lc.csv"
    with open(lc_file, 'w') as f:
        f.write('loop_count,res_t\n')

    rospy.init_node('ndt_stat_logger')
    rospy.Subscriber('/ndt_stat', NDTStat, ndt_stat_cb, lc_file)

    rospy.spin()