import numpy as np
import leg
'''
l = leg.leg(3,[1,1,1])

for s in l.segments:
	print(s.base,s.tip)
vecs = l.get_3D_vecs()

for v in vecs:
	print(v)

angs = l.get_angles_deg()
for a in angs:
	print(a)

l.follow([1+np.sqrt(2),0,np.sqrt(2)])

for s in l.segments:
	print(s.base,s.tip)
vecs = l.get_3D_vecs()

for v in vecs:
	print(v)

angs = l.get_angles_deg()
for a in angs:
	print(a)


'''
l2 = leg.leg(3,[1,1,1])

for s in l2.segments:
	print(s.base,s.tip)
endpoints = l2.get_3D_endpoints()

for p in endpoints:
	print(p)

angs = l2.get_angles_deg()
for a in angs:
	print(a)

l2.follow_lsq([1+np.sqrt(2),0,np.sqrt(2)])

for s in l2.segments:
	print(s.base,s.tip)
endpoints = l2.get_3D_endpoints()

for p in endpoints:
	print(p)

angs = l2.get_angles_deg(mode='sim')
for a in angs:
	print(a)
