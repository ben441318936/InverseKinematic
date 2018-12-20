import numpy as np

class segment(object):

	def __init__(self,x=0,y=0,len_=0,ang_=0):
		self.base = np.array([x,y])
		self.len = len_
		# Angle of a segment is with reference to horizontal
		self.ang = ang_
		self.calculate_tip()

	def calculate_tip(self,):
		tipX = self.base[0]+self.len*np.cos(self.ang)
		tipY = self.base[1]+self.len*np.sin(self.ang)
		self.tip = np.array([tipX,tipY])

	def set_angle(self,ang_):
		self.ang = ang_
		self.calculate_tip()

	def set_base(self,target):
		# target is a numpy vector
		self.base = target
		self.calculate_tip()

	def get_base(self):
		return self.base

	def get_tip(self):
		self.calculate_tip()
		return self.tip

	def get_angle(self):
		return self.ang

	def print_seg(self):
		print("Base: ",self.base,"Tip: ",self.tip,)
		return None

