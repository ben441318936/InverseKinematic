import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation

import leg

def update_legs(num, legs, lines, positions,prin=False):
	legs.follow_lsq(positions[num]);
	endpoints = legs.get_3D_endpoints()
	endpoints = np.array(endpoints)
	endpoints = np.transpose(endpoints)
	
	
	for line,ind in zip(lines,range(3)):
		line.set_data(endpoints[0:2,ind:ind+2])
		line.set_3d_properties(endpoints[2,ind:ind+2])
	
	'''
	lines.set_data(endpoints[0:2,:])
	lines.set_3d_properties(endpoints[2,:])
	'''

	if prin:
		angs = legs.get_angles_deg(mode='servo')
		for a in angs:
			print(a)
	return lines

positions = []

for t in range(50):
	positions.append([2+1*np.cos(t*np.pi/2/49),0,1*np.sin(t*np.pi/2/49)])

for t in range(70):
	positions.append([2-1*t/69,0,1+1*t/69])

for t in range(100):
	positions.append([1+2*np.cos(-1*t*np.pi/99+np.pi/2),0,2*np.sin(-1*t*np.pi/99+np.pi/2)])

for t in range(50):
	positions.append([1+2*np.cos(t*np.pi/2/49-np.pi/2),0,2*np.sin(t*np.pi/2/49-np.pi/2)])

legs = leg.leg(3,[1,1,1])
endpoints = legs.get_3D_endpoints()
endpoints = np.array(endpoints)

# Attaching 3D axis to the figure
fig = plt.figure()
ax = p3.Axes3D(fig)

# Creating fifty line objects.
# NOTE: Can't pass empty arrays into 3d version of plot()

#lines = ax.plot(endpoints[:,0],endpoints[:,1],endpoints[:,2],'ro-')[0]


lines = []
lines.append(ax.plot(endpoints[0:2,0],endpoints[0:2,1],endpoints[0:2,2],'g-',linewidth=5)[0])
lines.append(ax.plot(endpoints[1:3,0],endpoints[1:3,1],endpoints[1:3,2],'r-',linewidth=5)[0])
lines.append(ax.plot(endpoints[2:4,0],endpoints[2:4,1],endpoints[2:4,2],'b-',linewidth=5)[0])


# Setting the axes properties
ax.set_xlim3d([-3.0, 3.0])
ax.set_xlabel('X')

ax.set_ylim3d([-3.0, 3.0])
ax.set_ylabel('Y')

ax.set_zlim3d([-3.0, 3.0])
ax.set_zlabel('Z')

ax.set_title('Single Leg Test')

# Creating the Animation object
line_ani = animation.FuncAnimation(fig, update_legs, 270, fargs=(legs, lines, positions, False),
                              interval=0.1, blit=False)

plt.show()