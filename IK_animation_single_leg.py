import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import mpl_toolkits.mplot3d.proj3d as proj3d
import matplotlib.animation as animation

import leg

def update_legs(num, legs, lines, texts, ax, positions,prin=False):
	legs.follow_lsq(positions[num]);
	endpoints = legs.get_3D_endpoints()
	endpoints = np.array(endpoints)
	endpointsT = np.transpose(endpoints)
	
	
	for line,ind in zip(lines,range(3)):
		line.set_data(endpointsT[0:2,ind:ind+2])
		line.set_3d_properties(endpointsT[2,ind:ind+2])

	angs = legs.get_angles_deg(mode='servo')
	for text,ang,pos in zip(texts,angs,endpoints[0:3]):
		x,y,_ = proj3d.proj_transform(pos[0],pos[1],pos[2], ax.get_proj())
		text.set_position((x,y))
		text.set_text('%.3f'%(ang))
	
	'''
	lines.set_data(endpoints[0:2,:])
	lines.set_3d_properties(endpoints[2,:])
	'''

	if prin:
		angs = legs.get_angles_deg()
		for a in angs:
			print(a)
	return lines

positions = []

# Demo path
'''
for t in range(50):
	positions.append([2+1*np.cos(t*np.pi/2/49),0,1*np.sin(t*np.pi/2/49)])

for t in range(70):
	positions.append([2-1*t/69,0,1+1*t/69])

for t in range(100):
	positions.append([1*np.cos(t*2*np.pi/99),1*np.sin(t*2*np.pi/99),2])

for t in range(100):
	positions.append([1+2*np.cos(-1*t*np.pi/99+np.pi/2),0,2*np.sin(-1*t*np.pi/99+np.pi/2)])

for t in range(50):
	positions.append([1+2*np.cos(t*np.pi/2/49-np.pi/2),0,2*np.sin(t*np.pi/2/49-np.pi/2)])
'''

# Circular crawl path

for t in range(200):
	positions.append([1+np.sqrt(2),-1*np.sqrt(2)*np.cos(t*2*np.pi/199+np.pi/2),1*np.sqrt(2)*np.sin(t*2*np.pi/199+np.pi/2)])

legs = leg.leg(3,[1,1,1])
endpoints = legs.get_3D_endpoints()
endpoints = np.array(endpoints)

# Attaching 3D axis to the figure
fig = plt.figure()
ax = p3.Axes3D(fig)


# NOTE: Can't pass empty arrays into 3d version of plot()

#lines = ax.plot(endpoints[:,0],endpoints[:,1],endpoints[:,2],'ro-')[0]


lines = []
lines.append(ax.plot(endpoints[0:2,0],endpoints[0:2,1],endpoints[0:2,2],'-',linewidth=5,color=(0.5,0.5,1))[0])
lines.append(ax.plot(endpoints[1:3,0],endpoints[1:3,1],endpoints[1:3,2],'-',linewidth=5,color=(1,0.5,0.5))[0])
lines.append(ax.plot(endpoints[2:4,0],endpoints[2:4,1],endpoints[2:4,2],'-',linewidth=5,color=(0.5,1,0.5))[0])

texts = []
texts.append(ax.text2D(endpoints[0,0],endpoints[0,1],"test",color='black',fontsize=15,weight='bold'))
texts.append(ax.text2D(endpoints[1,0],endpoints[1,1],"test",color='black',fontsize=15,weight='bold'))
texts.append(ax.text2D(endpoints[3,0],endpoints[3,1],"test",color='black',fontsize=15,weight='bold'))


# Setting the axes properties
ax.set_xlim3d([-3.0, 3.0])
ax.set_xlabel('X')

ax.set_ylim3d([-3.0, 3.0])
ax.set_ylabel('Y')

ax.set_zlim3d([-3.0, 3.0])
ax.set_zlabel('Z')

ax.set_title('Single Leg Test')

# Creating the Animation object
line_ani = animation.FuncAnimation(fig, update_legs, 200, fargs=(legs, lines, texts, ax, positions, False),
                              interval=0.1, blit=False)

plt.show()