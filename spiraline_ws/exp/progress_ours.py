from os import kill, getenv, system
from signal import SIGTERM
from time import sleep
import subprocess
import argparse

def clean(pid_list):
    for pid in pid_list:
        kill(pid, SIGTERM)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='')
    parser.add_argument('--map', '-m', type=str, default='CubeTown_Obstacle')
    parser.add_argument('--rviz', '-r', action='store_true')
    parser.add_argument('--time', '-t', type=int, default=100)
    parser.add_argument('--exp', '-e', type=str, default='ndt_ours')
    args = parser.parse_args()

    spiraline_ws = getenv("HOME") + "/spiraline_ws"
    pid_list = []

    ### rosbridge
    try:
        rosbridge = subprocess.Popen([
            'roslaunch',
            'rosbridge_server',
            'rosbridge_websocket.launch'
            ])
        pid_list.append(rosbridge.pid)
        print('[System] rosbridge starts!')
        sleep(2)
    except Exception as e:
        print(e)
        print('[System] rosbridge fail!')
        clean(pid_list)
        exit(1)

    ### Open SVL script
    try:
        svl_script_process = subprocess.Popen([
            'python3',
            spiraline_ws + '/svl_script/' + args.map +'.py'
            ])
        pid_list.append(svl_script_process.pid)
        print('[System] SVL script open!')
        sleep(2)
    except Exception as e:
        print(e)
        print('[System] SVL script fail!')
        clean(pid_list)
        exit(1)

    ### drive_progress_logger
    try:
        drive_progress_logger = subprocess.Popen([
            'python3',
            spiraline_ws + '/util/drive_progress_logger.py',
            '-o',
            'ours.csv'
            ])
        pid_list.append(drive_progress_logger.pid)
        print('[System] Drive Progress Logger starts!')
        sleep(2)
    except Exception as e:
        print(e)
        print('[System] Drive Progress Logger fail!')
        clean(pid_list)
        exit(1)
    
    ### autorunner
    try:
        autorunner = subprocess.Popen([
            'roslaunch',
            'autorunner',
            'cubetown_autorunner.launch',
            '_exp:=' + args.exp
            ])
        pid_list.append(autorunner.pid)
        print('[System] autorunner starts!')
        sleep(2)
    except Exception as e:
        print(e)
        print('[System] autorunner fail!')
        clean(pid_list)
        exit(1)

    ### Rviz
    if args.rviz:
        try:
            rviz = subprocess.Popen([
                'rviz',
                '-d',
                spiraline_ws + '/cfg/rviz_config.rviz'
                ])
            pid_list.append(rviz.pid)
            print('[System] Rviz starts!')
            sleep(2)
        except Exception as e:
            print(e)
            print('[System] Rviz fail!')
            clean(pid_list)
            exit(1)
    
    # Wait for driving
    sleep(args.time)

    clean(pid_list)

    print("[System] Exp terminated successfully")

    ### Visualize
    system('python3 ' + spiraline_ws + '/util/driving_progress_viz.py -n ndt.csv -o ours.csv')

    print("[System] Save result into file!")