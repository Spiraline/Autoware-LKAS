import matplotlib.pyplot as plt
from os import getenv, makedirs, listdir
import csv

if __name__ == '__main__':

  csv_log_dir = getenv("HOME") + "/spiraline_ws/log/res_t/"
  res_dir = getenv("HOME") + "/spiraline_ws/res"
  makedirs(res_dir, exist_ok=True)

  fig, ax = plt.subplots()
  table_data = []

  for file in sorted(listdir(csv_log_dir)):
    node_name = file.split('.')[0]
    if node_name == "ndt_matching":
      continue
    exec_arr = []
    with open(csv_log_dir + file, 'r') as f:
      rr = csv.reader(f)
      for row in rr:
        exec_t = float(row[1]) - float(row[0])
        if len(exec_arr) > 10:
          exec_arr.sort()
          if exec_arr[0] < exec_t:
            del exec_arr[0]
            exec_arr.append(exec_t)
        else:
          exec_arr.append(exec_t)
      # print(node_name, [round(e*1000, 2) for e in exec_arr])
      wcet = exec_arr[0]
      for candidate in exec_arr[1:]:
        if candidate < wcet * 1.5:
          wcet = candidate
    table_data.append([node_name, round(wcet*1000, 2)])

  column_labels=["Node Name", "WCET (ms)"]
  ax.axis('tight')
  ax.axis('off')
  ax.table(cellText=table_data,colLabels=column_labels,loc="center")

  # plt.show()
  fig.savefig(res_dir + "/table1.png")