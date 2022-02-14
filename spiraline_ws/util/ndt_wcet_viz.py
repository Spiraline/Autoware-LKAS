import numpy as np
import matplotlib.pyplot as plt
from os import getenv, makedirs
from os.path import exists

plt.rcParams['axes.grid'] = False

csv_path = getenv("HOME") + "/spiraline_ws/log/ndt/lc.csv"

if not exists(csv_path):
  print('csv not exists!')
  exit(1)
  
lc_dict = {}

with open(csv_path, 'r') as f:
  f.readline()
  for row in f.readlines():
    lc, exec_t = int(row[:-2].split(',')[0]), float(row[:-2].split(',')[1])

    if lc == 0:
      continue

    if lc-1 not in lc_dict:
      lc_dict[lc-1] = []
    lc_dict[lc-1].append(exec_t)

fig, ax = plt.subplots()

x = [x for x in sorted(list(lc_dict.keys())) if x < 12]
y = [max(lc_dict[xx]) for xx in x]

plt.scatter(x, y)

x.extend([0 for i in range(1000)])
y.extend([0 for i in range(1000)])

fitted = np.polyfit(x, y, 1)
trend = np.poly1d(fitted)
slope = fitted[0]

x = x[:-1000]
y = y[:-1000]

ax.plot(x, y, color='black', label='Execution Time')
ax.plot(x, trend(x), color='black', linestyle=':', label='Trend Line \n(y = %.2fx)' % slope)

ax.set_xlabel('Loop Count')
ax.set_ylabel('Worst-Case Execution Time (ms)')

plt.xticks(range(0, 14, 2))
plt.legend()
# plt.show()
makedirs(getenv("HOME") + "/spiraline_ws/res", exist_ok=True)
fig.savefig(getenv("HOME") + "/spiraline_ws/res/fig13.png")