import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation
from scipy.io import loadmat

fig = plt.figure()
ax = p3.Axes3D(fig)

#R is 3 x 3 x numT array
#columns of R are the positions of the basis vectors
R=loadmat('Data/R.mat')
R=R['R']

lines = [ax.plot([0,0],[0,0],[0,1.],lw=3)[0],
ax.plot([0,0],[0,1],[0,0],lw=3)[0],
ax.plot([0,1],[0,0],[0,0],lw=3)[0]]

def update_lines(index,R,lines):
	for x in xrange(3):
		lines[x].set_data([0,R[x,0,index]],[0,R[x,1,index]])
		lines[x].set_3d_properties([0,R[x,2,index]])
	return lines

ax.set_xlim3d([-1.0, 1.0])
ax.set_xlabel('X')

ax.set_ylim3d([-1.0, 1.0])
ax.set_ylabel('Y')

ax.set_zlim3d([-1.0, 1.0])
ax.set_zlabel('Z')

ani = animation.FuncAnimation(fig,update_lines,
	fargs=(R,lines),interval=50,blit=False)

plt.show()