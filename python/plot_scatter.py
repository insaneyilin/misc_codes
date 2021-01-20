import os
import sys
import datetime
import numpy as np
import matplotlib.pyplot as plt
from textwrap import wrap

x = np.linspace(0, 2 * np.pi, 100)

sin_x = np.sin(x)
cos_x = np.cos(x)
x_with_noise = x + np.random.normal(0.0, 0.5, len(x))

sin_x_plt = plt.scatter(
    x, sin_x, s=0.5)
cos_x_plt = plt.scatter(
    x, cos_x, s=0.5)
x_with_noise_plt = plt.scatter(
    x, x_with_noise, s=0.5)

plt.legend((sin_x_plt, cos_x_plt, x_with_noise_plt),
           ('y = sin(x)', 'y = cos(x)', 'y = x with noise'),
           scatterpoints=1,
           loc='upper left',
           ncol=3,
           fontsize=8)

plt.title("scatter plot")
plt.xlabel("x label")
plt.ylabel("y label")
plt.title("\n".join(
    wrap("very very very very very very very very very very very very very very long title", 60)))
plt.show()

# save plot
# plt.savefig(output_img_path)
