import numpy as np
import matplotlib.pyplot as plt
import re

file = open("../relative_distance_file.txt").readlines()

distances = []

for line in file:
    line = re.split(",|\n", line)
    print(line)
    distances.append(float(line[0]))

plt.plot(distances)
plt.show()
