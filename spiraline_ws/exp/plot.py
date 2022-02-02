from os import system, getenv

spiraline_ws = getenv("HOME") + "/spiraline_ws"

### Fig 1b
if system('python3 ' + spiraline_ws + '/util/acc_viz.py') == 0:
    print("[System] Fig1b saved")

### Fig 13
if system('python3 ' + spiraline_ws + '/util/ndt_wcet_viz.py') == 0:
    print("[System] Fig13 saved")

### Fig15b
if system('python3 ' + spiraline_ws + '/util/driving_progress_viz.py -n ndt.csv -o ours.csv') == 0:
    print("[System] Fig15 saved")