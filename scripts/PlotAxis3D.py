# Created by steve on 17-6-15 上午8:05


import numpy as np
import scipy as sp
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3

class PlotAxis3D:
    '''
    Plot axis according to 9-d vector(each vector represents a single axis),
    or with 12-d vector(include the offset)

    '''
    def __init__(self,ax,file_name,axis_len=0.1):
        data = np.loadtxt(file_name)
        for i in range(data.shape[0]):
            for j in range(3):
                tmp_axis = np.zeros([2,3])
                tmp_axis[0,:] = data[i,:3]
                tmp_axis[1,:] = data[i,:3] + data[i,3+j*3:6+j*3]/\
                                             np.linalg.norm(data[i,3+j*3:6+j*3])*axis_len

                the_color = (0,1.0,1.0)
                # the_color[j] = 1.0
                ax.plot(tmp_axis[:,0],tmp_axis[:,1],tmp_axis[:,2],color=the_color)






if __name__ == '__main__':
    fig = plt.figure()
    ax = p3.Axes3D(fig)
    pa = PlotAxis3D(ax,"../ResultData/out_axis.txt",0.1)
    plt.show()
