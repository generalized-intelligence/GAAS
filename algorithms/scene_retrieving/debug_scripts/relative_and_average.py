import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import re

file = open("../relative_and_average_file.txt").readlines()
file = file[3:]
length = len(file)

for line in file[3:]:
    line = re.split("\n|,", line)
    print(line)

print(length)