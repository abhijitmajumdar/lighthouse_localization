import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os

def plot_pose(fname,offset_start=None,offset_end=None,xlims=None,ylims=None):
    if not os.path.isfile(fname):
        raise ValueError("File not found! Please create pose data file first")
    pose_arr = np.load(fname)
    data_offset = (offset_start,offset_end)
    x = pose_arr[data_offset[0]:data_offset[1],0,3]
    y = pose_arr[data_offset[0]:data_offset[1],1,3]
    z = pose_arr[data_offset[0]:data_offset[1],2,3]
    t = list(range(len(x)))
    t = [_t/50.0 for _t in t]
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    points = plt.scatter(x,y,zs=z,s=5,c=t, cmap='plasma', label="Trajactory")
    # Add a color bar which maps values to colors.
    cbar = fig.colorbar(points, shrink=0.5, aspect=5)
    cbar.ax.set_ylabel("Time (seconds)")
    # plt.title('Autonomous Drone Trajactory - square pattern')
    ax.set_xlabel('X-axis (meter)')
    ax.set_xlim(-0.6, 0.6)
    ax.set_ylabel('Y-axis (meter)')
    ax.set_ylim(-0.6, 0.6)
    ax.set_zlabel('Z-axis (meter)')
    ax.set_zlim(0, 1.0)
    plt.legend()
    plt.show()

def save_pose_data(p,fname):
    print "Recorded",len(p)," data points"
    np.save(fname+".npy",p)
    print "Saved to",fname
