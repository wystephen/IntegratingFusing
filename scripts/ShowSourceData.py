# Created by steve on 17-6-11 下午10:01


import numpy as np
import matplotlib.pyplot as plt


data = np.loadtxt("/home/steve/Code/Mini_IMU/Scripts/IMUWB/47/imu.txt",
                  delimiter=',')

plt.figure()
print(data[:,1:4].shape)
print(np.mean(data[:20,1:4]*9.81,axis=0))
print(np.linalg.norm( np.mean(data[:20,1:4]*9.81,axis=0)))
print(np.mean(data[:20,1:4],axis=0))
print(np.linalg.norm( np.mean(data[:20,1:4],axis=0)))
plt.plot(np.linalg.norm(data[:,1:4],axis=1)*9.81,'r')
plt.show()