import numpy as np
import segment as sg

class leg(object):

	def __init__ (self,num_segs,lens,beta1_=0.8,beta2_=2):
		# lens is a list of leg segment lengths
		self.segments = [sg.segment(0,0,lens[0],0)]
		for i in range(1,num_segs):
			self.segments.append(sg.segment(self.segments[i-1].get_base()[0]+lens[i-1],0,lens[i],0))
		self.beta1 = beta1_
		self.beta2 = beta2_



	def follow (self,target):
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
		for i in range(5000):
			self.follow_in_theta_plane(tar_theta_plane)

		return None


	def follow_in_theta_plane(self,tar_theta_plane):
		self.segments[-1].follow(tar_theta_plane)
		#print("Final segment after following:")
		#self.segments[-1].print_seg()

		i = len(self.segments)-2
		while i > 0:
			self.segments[i].follow(self.segments[i+1].get_base())
			i = i-1

		
		#print("Middle segment after following:")
		#self.segments[1].print_seg()

		self.segments[1].set_base(np.array([self.segments[0].len,0]))

		#print("Middle segment after rebasing: ")
		#self.segments[1].print_seg()

		i = 2
		while i < len(self.segments):
			self.segments[i].set_base(self.segments[i-1].get_tip())
			i = i+1

		#print("Finall segment after rebasing: ")
		#self.segments[2].print_seg()

		return None

	def follow_lsq (self,target):
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
		self.follow_in_theta_plane_lsq(tar_theta_plane)

		return None

	def follow_in_theta_plane_lsq(self,tar_theta_plane):
		n = len(self.segments)
		x = np.zeros(n-1)
		f = self.compute_f(x,tar_theta_plane)
		#print("f:",f)
		lamb = 0.1
		# Using non-linear least-square optimization
		# Levenberg-Marquis update
		# More iterations better results
		for i in range(500):
			delta_x = self.compute_delta_x(x,lamb,tar_theta_plane)
			if np.linalg.norm(delta_x) < 1e-6:
				#print("i:",i)
				break
			x_hat = x - delta_x

			if np.linalg.norm(self.compute_f(x_hat,tar_theta_plane)) < np.linalg.norm(self.compute_f(x,tar_theta_plane)):
				x = x_hat
				lamb = self.beta1*lamb
			else: 
				x = x
				lamb = self.beta2*lamb		

		for i in range(1,n):
			self.segments[i].set_angle(np.sum(x[0:i]))
			if i < n-1:
				self.segments[i+1].set_base(self.segments[i].get_tip())

		return None

	def compute_f(self,x,des_pt):
		n = len(self.segments)
		f = np.zeros(n-1)
		f[0] = self.segments[0].len
		ang = 0
		for i in range(1,n):
			ang = np.sum(x[0:i])
			f[0] = f[0] + self.segments[i].len * np.cos(ang)
			f[1] = f[1] + self.segments[i].len * np.sin(ang)
		f[0] = f[0] - des_pt[0]
		f[1] = f[1] - des_pt[1]

		return f

	def compute_delta_x(self,x,lamb,des_pt):
		n = len(self.segments)
		grad = np.zeros((2,n-1))
		for i in range(1,n):
			grad_1 = 0
			grad_2 = 0
			ang = 0
			for j in range(i,n):
				ang = np.sum(x[0:j])
				grad_1 = grad_1 + self.segments[j].len * -1 * np.sin(ang)
				grad_2 = grad_2 + self.segments[j].len * np.cos(ang)
			grad[0,i-1] = grad_1
			grad[1,i-1] = grad_2

		a = np.matmul( np.transpose(grad) , grad)
		b = a + lamb*np.identity(n-1)
		c = np.linalg.inv(b)
		d = np.matmul( c, np.transpose(grad))
		delta_x = np.matmul( d , self.compute_f(x,des_pt))

		return delta_x 


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
			for i in range(1,len(results)):
				results[i] = results[i] - results[i-1]
		elif mode != 'sim':
			pass

		return np.array(results)

	def get_angles_deg(self,mode='servo'):
		ang = self.get_angles(mode)

		return ang/np.pi*180




