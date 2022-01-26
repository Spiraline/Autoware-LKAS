import argparse
import pandas as pd
import matplotlib.pyplot as plt
from os import getenv, makedirs
from os.path import exists



if __name__ == '__main__':
  parser = argparse.ArgumentParser(description='')
  parser.add_argument('--ndt', '-n', type=str, required=True, help='original NDT log file name')
  parser.add_argument('--ours', '-o', type=str, required=True, help='ours log file name')
  parser.add_argument('--time', '-t', type=int, default=15, help='plot until this time')
  parser.add_argument('--time_wall', '-w', type=int, default=57, help='time wall')
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

  time_index = args.time * 10

  ax.plot([t/10 for t in range(time_index)], original_case['center_offset'][0:time_index], color='gray', linestyle='-', label='Original Autoware\n(L=30)')
  ax.plot([t/10 for t in range(time_index)], ours_case['center_offset'][0:time_index], color='black', linestyle='--', label='Our modified\nAutoware')

  ax.set_ylabel('Center Offset (m)')
  ax.set_xlabel('Time (s)')

  ax.set_ylim(-20, 21)
  plt.xticks(range(0, args.time, 2))

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

  plt.xticks(range(0, args.time, 2))

  plt.legend()
  plt.show()
  fig.savefig(res_dir + "/fig15b.png")