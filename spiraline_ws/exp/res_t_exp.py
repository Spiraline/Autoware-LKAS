from os import fork, kill, getenv
from signal import SIGTERM
import roslaunch, rospkg
from time import sleep
import subprocess

def launch_script(pkg, node, args=""):
    pid = fork()

    if pid == 0:
        launch_file = roslaunch.core.Node(
            package=pkg,
            node_type=node,
            args=args,
            output='screen')
        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()
        process = launch.launch(launch_file)
        process.stop()
        exit(0)
    return pid

def clean(pid_list):
    for pid in pid_list:
        kill(pid, SIGTERM)

if __name__ == "__main__":
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
            spiraline_ws + '/svl_script/CubeTown_Obstacle.py'
            ])
        pid_list.append(svl_script_process.pid)
        print('[System] SVL script open!')
        sleep(2)
    except Exception as e:
        print(e)
        print('[System] SVL script fail!')
        clean(pid_list)
        exit(1)
    
    ### autorunner
    try:
        autorunner = subprocess.Popen([
            'roslaunch',
            'autorunner',
            'cubetown_autorunner.launch',
            '_exp:=autoware'
            ])
        pid_list.append(autorunner.pid)
        print('[System] autorunner starts!')
        sleep(2)
    except Exception as e:
        print(e)
        print('[System] autorunner fail!')
        clean(pid_list)
        exit(1)
    
    # Wait for driving
    sleep(1000)

    clean(pid_list)

    print("[System] Exp terminated successfully")
    