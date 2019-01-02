import numpy as np
import segment as sg

'''
A leg is a collection of 2D segments. Every position in 3D is viewed in cylindrical coordinates, 
which allows us to use the first segment in horizontal plane, and the rest of the segments in the plane specified 
by an angle from x-axis. 
'''

class leg(object):

	def __init__ (self,num_segs,lens,beta1_=0.8,beta2_=2):
		# lens is a list of leg segment lengths
		self.segments = [sg.segment(0,0,lens[0],0)]
		for i in range(1,num_segs):
			self.segments.append(sg.segment(self.segments[i-1].get_base()[0]+lens[i-1],0,lens[i],0))

		# Used for non-linear least-squares
		self.beta1 = beta1_
		self.beta2 = beta2_

	def follow_lsq(self,target):
		# target is a 3D numpy vector
		# convert cartesian to cylindrical
		x = target[0]
		y = target[1]
		z = target[2]
		theta = np.arctan2(y,x)
		rho = np.linalg.norm([x,y])

		# set angle for base leg
		self.segments[0].set_angle(theta)

		# follow in the theta plane using the remain segments
		tar_theta_plane = np.array([rho,z])
		#print("Plane target:",rho,z)

		# Compute new angle, using LM damped by angle change
		self.follow_in_theta_plane_lsq(tar_theta_plane)

		return None

	def follow_in_theta_plane_lsq(self,tar_theta_plane):

		# PARAMs to tune: 
		# 	angle change tolerance
		#	iterations of damped LM
		#	damping factor

		ang_change_tol = np.pi/18
		damped_LM_iters = 3
		damping_factor = 0.15

		n = len(self.segments)
		prev_angs = self.get_angles(mode='servo')[1:]
		# Using Levenberg–Marquardt algorithm on only location objective
		x = self.LM_algo(tar_theta_plane,prev_angs,gamma=0)
		
		# Compare results to previous angles, if change is large, use multi-objective LM
		for i in range(0,n-1):
			if np.abs(x[i]-prev_angs[i]) > ang_change_tol:
				large_change = True
				break
			else:
				large_change = False

		if large_change:
			# More iterations better results
			# gamma controls the relative importance of angle change
			for k in range(0,damped_LM_iters):
				prev_angs = self.get_angles(mode='servo')[1:]
				x = self.LM_algo(tar_theta_plane,prev_angs,gamma=damping_factor)

				# Set the segment angles using the non-linear least-squares solution
				for i in range(1,n):
					self.segments[i].set_angle(np.sum(x[0:i]))
					if i < n-1:
						self.segments[i+1].set_base(self.segments[i].get_tip())
		else:
			# Set the segment angles using the non-linear least-squares solution
			for i in range(1,n):
				self.segments[i].set_angle(np.sum(x[0:i]))
				if i < n-1:
					self.segments[i+1].set_base(self.segments[i].get_tip())
		

		return None

	def LM_algo(self,tar_theta_plane,prev_angs,gamma=0):
		n = len(self.segments)
		x = np.zeros(n-1)
		f = self.compute_f(x,tar_theta_plane,prev_angs,gamma)
		#print("f:",f)
		lamb = 0.1
		# More iterations better results
		for i in range(100):
			delta_x = self.compute_delta_x(x,lamb,tar_theta_plane,prev_angs,gamma)
			if np.linalg.norm(delta_x) < 1e-6:
				#print("i:",i)
				break
			x_hat = x - delta_x
			# Check if function value actually decreases
			if np.linalg.norm(self.compute_f(x_hat,tar_theta_plane,prev_angs,gamma)) < np.linalg.norm(self.compute_f(x,tar_theta_plane,prev_angs,gamma)):
				x = x_hat
				lamb = self.beta1*lamb
			else: 
				x = x
				lamb = self.beta2*lamb	

		return x

	# Computes the actual function value that we are minimizing at the current step
	def compute_f(self,x,des_pt,prev_angs,gamma=0):
		n = len(self.segments)
		f = np.zeros(2*(n-1))
		f[0] = self.segments[0].len
		ang = 0
		for i in range(1,n):
			ang = np.sum(x[0:i])
			f[0] = f[0] + self.segments[i].len * np.cos(ang)
			f[1] = f[1] + self.segments[i].len * np.sin(ang)
		f[0] = f[0] - des_pt[0]
		f[1] = f[1] - des_pt[1]
		f[2:] = gamma*(x - prev_angs)

		return f

	# Computes update at current step
	def compute_delta_x(self,x,lamb,des_pt,prev_angs,gamma=0):
		# Gamma controls the relative weight of ensuring small angle changes
		n = len(self.segments)
		jab = np.zeros((2*(n-1),n-1))
		for i in range(1,n):
			# jab_i is the ith row of the Jacobian matrix
			jab_1 = 0
			jab_2 = 0
			ang = 0
			for j in range(i,n):
				ang = np.sum(x[0:j])
				jab_1 = jab_1 + self.segments[j].len * -1 * np.sin(ang)
				jab_2 = jab_2 + self.segments[j].len * np.cos(ang)
			jab[0,i-1] = jab_1
			jab[1,i-1] = jab_2
		jab[2,0] = np.sqrt(gamma)
		jab[2,1] = 0
		jab[3,0] = 0
		jab[3,1] = np.sqrt(gamma)

		a = np.matmul( np.transpose(jab) , jab)
		b = a + lamb*np.identity(n-1)
		c = np.linalg.inv(b)
		d = np.matmul( c, np.transpose(jab))

		delta_x = np.matmul( d , self.compute_f(x,des_pt,prev_angs,gamma))

		return delta_x 

	# Returns n+1 points
	def get_3D_endpoints(self):
		results = [np.array([0,0,0])]
		results.append(np.array([self.segments[0].get_tip()[0],self.segments[0].get_tip()[1],0]))

		i = 2
		while i <= len(self.segments):
			x = self.segments[i-1].get_tip()[0]*np.cos(self.segments[0].get_angle())
			y = self.segments[i-1].get_tip()[0]*np.sin(self.segments[0].get_angle())
			z = self.segments[i-1].get_tip()[1]
			results.append(np.array([x,y,z]))
			i = i+1

		return results

	def get_angles(self,mode):
		results = []
		for s in self.segments:
			results.append(s.get_angle())
		if mode == 'servo':
			# If values are sent to servo, the angles are with reference to each joint
			for i in range(2,len(results)):
				results[i] = results[i] - results[i-1]
		elif mode != 'sim':
			# If values are for simulation output, we can show angles with reference to horizontal
			pass

		return np.array(results)

	def get_angles_deg(self,mode='servo'):
		ang = self.get_angles(mode)

		return ang/np.pi*180




