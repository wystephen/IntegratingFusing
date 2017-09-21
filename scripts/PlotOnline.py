# Created by steve on  下午6:53

import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation

import time
import os


class RealTimePlot:
    def __init__(self, file_name):
        self.file_name = file_name
        self.n_time = os.stat(file_name).st_mtime

        self.fig = plt.figure()
        self.ax = p3.Axes3D(self.fig)

        self.data = np.loadtxt(self.file_name)
        self.ax.plot(self.data[:, 0], self.data[:, 1], self.data[:, 2], '*r-')

    def start(self):
        line_ani = animation.FuncAnimation(
            self.fig, self.update_trace, interval=100
        )
        plt.show()

    def update_trace(self, data):
        # print("all in here")
        # print(os.stat(file_name).st_atime)

        if (os.stat(file_name).st_mtime > self.n_time):

            time.sleep(1)
            self.n_time = os.stat(self.file_name).st_mtime
            self.data = np.loadtxt(self.file_name)
            self.ax.clear()
            if self.data.shape[0]>1:
                self.ax.plot(self.data[:, 0],
                             self.data[:, 1],
                             self.data[:, 2], '-*')


if __name__ == '__main__':
    file_name = '../ResultData/out_result.txt'
    # print(os.stat(file_name).st_mtime)

    rshow = RealTimePlot(file_name)

    rshow.start()
