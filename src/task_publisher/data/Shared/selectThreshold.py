import numpy as np
import matplotlib.pyplot as plt

id = 1
force = np.loadtxt('humanForce_id' + str(id) + '.txt')
forceNorm = np.linalg.norm(force, axis=1)
deviation = np.loadtxt('deviation_id' + str(id) + '.txt')

timeLen = min(forceNorm.shape[0], deviation.shape[0])

forceNorm = forceNorm[0:timeLen]
deviation = deviation[0:timeLen]

# 上下子图
fig, ax1 = plt.subplots()
ax2 = ax1.twinx()
ax1.plot(forceNorm, 'g-')
ax2.plot(deviation, 'b-')

ax1.set_xlabel('time')
ax1.set_ylabel('forceNorm', color='g')
ax2.set_ylabel('deviation', color='b')
ax2.set_ylim(0, 0.04)

plt.show()