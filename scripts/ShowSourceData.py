# Created by steve on 17-6-11 下午10:01


import numpy as np
import matplotlib.pyplot as plt

data = np.loadtxt("/home/steve/Code/Mini_IMU/Scripts/IMUWB/46/imu.txt",
                  delimiter=',')

plt.figure()
print(data[:, 1:4].shape)
print(np.mean(data[:20, 1:4] * 9.81, axis=0))
print(np.linalg.norm(np.mean(data[:20, 1:4] * 9.81, axis=0)))
print(np.mean(data[:20, 1:4], axis=0))
print(np.linalg.norm(np.mean(data[:20, 1:4], axis=0)))
plt.plot(np.linalg.norm(data[:, 1:4], axis=1) * 9.81, 'r')
plt.figure()
plt.plot(np.linalg.norm(data[:, 4:7], axis=1) * np.pi / 180.0, 'g')
plt.show()

tmp_data = data[:, :7]
print(tmp_data.shape)
tmp_data[:, 0] -= tmp_data[0, 0]
tmp_data[:, 1:4] *= 9.81
tmp_data[:, 4:7] *= (3.1415 / 180.0)
np.savetxt("test.csv", tmp_data, delimiter=';')
