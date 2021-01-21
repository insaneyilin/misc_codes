import os
import sys
import datetime
import numpy as np
import matplotlib.pyplot as plt
from collections import defaultdict
from textwrap import wrap


frequency_dict = defaultdict(int)
frequency_dict[0] = 42
frequency_dict[1] = 10
frequency_dict[2] = 23
frequency_dict[3] = 19
frequency_dict[4] = 6

plt.bar(frequency_dict.keys(), frequency_dict.values(), width=0.5, color='g')

# or use np.histogram & plt.hist
plt.figure()  # create a new figure
arr = np.array([22, 87, 5, 43, 56, 73, 55, 54, 11, 20, 51, 5, 79, 31, 27])
np.histogram(arr, bins = [0, 20, 40, 60, 80, 100])
hist, bins = np.histogram(arr, bins =  [0, 20, 40, 60, 80, 100])
plt.hist(arr, bins=bins)
plt.show()
