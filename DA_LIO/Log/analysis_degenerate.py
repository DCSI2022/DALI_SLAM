import numpy as np
import matplotlib.pyplot as plt
import sys

filename = sys.argv[1]
data = np.loadtxt(filename)

timestamp = data[:,0]
offset = timestamp[0]
for i in range(len(timestamp)):
    timestamp[i] = timestamp[i] - offset

mini_eigen = data[:,1]
plt.hist(mini_eigen, bins=128)
#plt.plot(timestamp,mini_eigen, linewidth=1.5)
plt.show()
