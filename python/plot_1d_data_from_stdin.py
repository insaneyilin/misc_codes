import os
import sys
import datetime
import numpy as np
import matplotlib.pyplot as plt


def isfloat(value):
    try:
        float(value)
        return True
    except ValueError:
        return False


data = []

if not sys.stdin.isatty():
    for line in sys.stdin:
        line = line.strip()
        if not isfloat(line):
            continue
        x = float(line)
        data.append(x)

len_data = len(data)
s_idx = int(0.01 * len_data)
e_idx = int(0.99 * len_data)

data = np.array(data)

data = data[s_idx:e_idx]
len_data = len(data)

data_mean = np.mean(data)
data_mean_list = [data_mean] * len(data)

data_stddev = np.std(data)

fig, ax = plt.subplots()

data_line = ax.plot(data, label='1D data', marker='o')
mean_line = ax.plot(data_mean_list, label='Mean', linestyle='--')

legend = ax.legend(loc='upper right')
plt.title("len: {} mean: {} stddev: {}".format(
    len_data, data_mean, data_stddev))
plt.show()
