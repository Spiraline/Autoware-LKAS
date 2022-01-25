import matplotlib.pyplot as plt
from os import getenv, makedirs
from os.path import exists

csv_path = getenv("HOME") + "/spiraline_ws/log/ndt/accuracy.csv"

if not exists(csv_path):
    print('csv not exists!')
    exit(1)

acc_dict = {}

with open(csv_path, 'r') as f:
    for row in f.readlines():
        acc = [1 - float(e) for e in row[:-2].split(',')]
        if len(acc) not in acc_dict:
            acc_dict[len(acc)] = []
        acc_dict[len(acc)].append(acc)

# plt.rcParams['axes.grid'] = False
plt.rcParams["figure.figsize"] = (7,5)
plt.rcParams["font.size"] = 12

fig, ax = plt.subplots()

plt.axhline(1-0.01, color='gray', lw=2)

ax.set_ylabel('Accuracy')
ax.set_xlabel('Loop Count')

lc_list = sorted(list(acc_dict.keys()))

plt.plot(acc_dict[lc_list[1]][0], color='black', linestyle='-', label='Case 1')
plt.plot(acc_dict[lc_list[-1]][0], color='black', linestyle='-.', label='Case 2')
if len(lc_list) > 4:
    plt.plot(acc_dict[lc_list[len(lc_list)//2]][0], color='black', linestyle=':',  label='Case 3')

plt.legend()

plt.show()
makedirs(getenv("HOME") + "/spiraline_ws/res", exist_ok=True)
fig.savefig(getenv("HOME") + "/spiraline_ws/res/fig1b_acc.png")
# fig.savefig("fig1b_acc.eps", format="eps")