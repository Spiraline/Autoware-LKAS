import argparse
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from os import getenv, makedirs
from os.path import exists

if __name__ == '__main__':
  parser = argparse.ArgumentParser(description='')
  parser.add_argument('--ndt', '-n', type=str, required=True, help='original NDT log file name')
  parser.add_argument('--ours', '-o', type=str, required=True, help='ours log file name')
  parser.add_argument('--time', '-t', type=int, default=40, help='plot until this time')
  parser.add_argument('--time_wall', '-w', type=int, default=56, help='time wall')
  args = parser.parse_args()

  csv_log_dir = getenv("HOME") + "/spiraline_ws/log/ndt/"
  res_dir = getenv("HOME") + "/spiraline_ws/res"
  makedirs(res_dir, exist_ok=True)
  original_file = csv_log_dir + args.ndt
  ours_file = csv_log_dir + args.ours

  if not exists(original_file) or not exists(ours_file):
    print('csv not exists!')
    exit(1)

  original_case = pd.read_csv(original_file)
  ours_case = pd.read_csv(ours_file)
  
  ### Center offset
  plt.rcParams["figure.figsize"] = (7,5)
  plt.rcParams["font.size"] = 11.5

  fig, ax = plt.subplots()

  plt.axhline(0, color='gray', lw=0.5, linestyle=(0, (1, 5)))

  plt.axhline(6, color='black', lw=2)
  plt.axhline(-6, color='black', lw=2)

  time_index = min(args.time * 10, len(original_case) // 10 * 10, len(ours_case) // 10 * 10)

  ax.plot([t/10 for t in range(time_index)], original_case['center_offset'][0:time_index], color='gray', linestyle='-', label='Original Autoware\n(L=30)')
  ax.plot([t/10 for t in range(time_index)], ours_case['center_offset'][0:time_index], color='black', linestyle='--', label='Our modified\nAutoware')

  ax.set_ylabel('Center Offset (m)')
  ax.set_xlabel('Time (s)')

  ax.set_ylim(-20, 20)
  plt.xticks(range(0, (time_index+1) // 10, 2))

  backup_period = []
  prev_state = 'Normal'
  start_t = 0
  for t in range(time_index):
    if ours_case['state'][t] != prev_state:
      if prev_state == 'Normal':
        start_t = t
      else:
        backup_period.append((start_t / 10, t / 10))
      prev_state = ours_case['state'][t]

  if prev_state == 'Backup':
    backup_period.append((start_t / 10, time_index / 10))

  for s, e in backup_period:
    ax.add_patch(patches.Rectangle((s, -20), (e-s), 40, facecolor=[0,0,0,0.1], fill=True))

  plt.legend()
  fig.savefig(res_dir + "/fig15a.png")

  ### Res_t
  fig, ax = plt.subplots()

  plt.axhline(0, color='gray', lw=0.5, linestyle=(0, (1, 5)))
  plt.axhline(args.time_wall, color='black', lw=2, linestyle='-')

  ax.plot([t/10 for t in range(time_index)], original_case['res_t'][0:time_index], color='gray', linestyle='-', label='Original Autoware\n(L=30)')
  ax.plot([t/10 for t in range(time_index)], ours_case['res_t'][0:time_index], color='black', linestyle='--', label='Our modified\nAutoware')

  ax.set_ylabel('Execution Time (ms)')
  ax.set_xlabel('Time (s)')

  ax.set_ylim(0, 250)

  plt.xticks(range(0, (time_index+1) // 10, 2))

  for s, e in backup_period:
    ax.add_patch(patches.Rectangle((s, 0), (e-s), 250, facecolor=[0,0,0,0.1], fill=True))

  plt.legend()
  # plt.show()
  fig.savefig(res_dir + "/fig15b.png")