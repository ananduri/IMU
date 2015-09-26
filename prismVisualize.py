import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from scipy.io import loadmat
from Quaternion import Quaternion


fig = plt.figure()
ax = fig.add_subplot(111,xlim=(-2.5,2.5),ylim=(-2.5,2.5),aspect='equal')

d = loadmat('Data/quat2.mat')
quat = d['quat']

# we want three kinds of faces
z_face = np.array([[2,0.2,1],[2,-0.2,1],[-2,-0.2,1],[-2,0.2,1],[2,0.2,1]])
x_face = np.array([[2,0.2,-1],[2,-0.2,-1],[2,-0.2,1],[2,0.2,1],[2,0.2,-1]])
y_face = np.array([[2,0.2,-1],[2,0.2,1],[-2,0.2,1],[-2,0.2,-1],[2,0.2,-1]])

# and reflections about the three axes
x, y, z = np.eye(3)
rots_x = [Quaternion.from_v_theta(x,theta) for theta in (np.pi,0)]
rots_y = [Quaternion.from_v_theta(y,theta) for theta in (np.pi,0)]
rots_z = [Quaternion.from_v_theta(z,theta) for theta in (np.pi,0)]

colors = ['blue', 'green', 'black', 'yellow', 'orange', 'red']

current_rot = Quaternion.from_v_theta((1, 0, 0), 0.01) #can be whatever

Rs_x = [(current_rot * rot).as_rotation_matrix() for rot in rots_x]
Rs_y = [(current_rot * rot).as_rotation_matrix() for rot in rots_y]
Rs_z = [(current_rot * rot).as_rotation_matrix() for rot in rots_z]

faces_x = [np.dot(x_face,R.T) for R in Rs_y]
faces_y = [np.dot(y_face,R.T) for R in Rs_z]
faces_z = [np.dot(z_face,R.T) for R in Rs_x]

faces_proj_x = [face[:, :2] for face in faces_x]
faces_proj_y = [face[:, :2] for face in faces_y]
faces_proj_z = [face[:, :2] for face in faces_z]

zorder_x = [face[:4, 2].sum() for face in faces_x] #interesting, works if the faces are connected
zorder_y = [face[:4, 2].sum() for face in faces_y]
zorder_z = [face[:4, 2].sum() for face in faces_z]

polys = [plt.Polygon(faces_proj_x[i], fc=colors[i],alpha=0.9, zorder=zorder_x[i]) for i in xrange(2)]
polys += [plt.Polygon(faces_proj_y[i], fc=colors[i+2], alpha=0.9, zorder=zorder_y[i]) for i in xrange(2)]
polys += [plt.Polygon(faces_proj_z[i], fc=colors[i+4], alpha=0.9, zorder=zorder_z[i]) for i in xrange(2)]

patchs = tuple(polys)

def init():
	for i in range(6):
		ax.add_patch(polys[i])
	return patchs,

# TODO: add constant rotation for viewing from different camera angle
def animate(i):
	q = quat[:,i]

	angle = 2*np.arccos(q[0])

	# catch when dividing by 0 because of angle
	if(q[0]==1.):
		axis=(0,0,1)
	else:
		axis = tuple(q[1:]/np.sqrt(1-(q[0]**2)))

	current_rot = Quaternion.from_v_theta(axis, angle)

	Rs_x = [(current_rot * rot).as_rotation_matrix() for rot in rots_x]
	Rs_y = [(current_rot * rot).as_rotation_matrix() for rot in rots_y]
	Rs_z = [(current_rot * rot).as_rotation_matrix() for rot in rots_z]

	faces_x = [np.dot(x_face,R.T) for R in Rs_y]
	faces_y = [np.dot(y_face,R.T) for R in Rs_z]
	faces_z = [np.dot(z_face,R.T) for R in Rs_x]

	faces_proj_x = [face[:, :2] for face in faces_x]
	faces_proj_y = [face[:, :2] for face in faces_y]
	faces_proj_z = [face[:, :2] for face in faces_z]

	zorder_x = [face[:4, 2].sum() for face in faces_x] 
	zorder_y = [face[:4, 2].sum() for face in faces_y]
	zorder_z = [face[:4, 2].sum() for face in faces_z]

	for i in range(2):
		polys[i].set_xy(faces_proj_x[i])
		polys[i].set_zorder(zorder_x[i])
		polys[i+2].set_xy(faces_proj_y[i])
		polys[i+2].set_zorder(zorder_y[i])
		polys[i+4].set_xy(faces_proj_z[i])
		polys[i+4].set_zorder(zorder_z[i])

	return tuple(polys)

#want blit=True
anim = animation.FuncAnimation(fig, animate, init_func=init,frames=1400, interval=40, blit=False) 

plt.show()