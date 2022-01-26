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
    wcet = 0
    with open(csv_log_dir + file, 'r') as f:
      rr = csv.reader(f)
      for row in rr:
        exec_t = float(row[1]) - float(row[0])
        if exec_t > wcet:
          wcet = exec_t
    table_data.append([node_name, round(wcet*1000, 2)])

  column_labels=["Node Name", "WCET (ms)"]
  ax.axis('tight')
  ax.axis('off')
  ax.table(cellText=table_data,colLabels=column_labels,loc="center")

  # plt.show()
  fig.savefig(res_dir + "/table1.png")