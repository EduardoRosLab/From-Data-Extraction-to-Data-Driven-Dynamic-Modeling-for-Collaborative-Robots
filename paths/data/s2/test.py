import numpy as np

import matplotlib.pyplot as plt

arr = np.fromfile("cartesian_1000")
arr = arr.reshape(len(arr) // 3, 3)

ax = plt.figure().add_subplot(projection="3d")

ax.plot(arr[:, 0], arr[:, 1], arr[:, 2])

plt.show()
