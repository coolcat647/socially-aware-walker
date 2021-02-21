# Author: Xinshuo Weng
# email: xinshuo.weng@gmail.com

import numpy as np
from filterpy.kalman import KalmanFilter

class KalmanBoxTracker(object):
	"""
	This class represents the internel state of individual tracked objects observed as bbox.
	"""
	count = 0
	def __init__(self, det2D, info):
		"""
		Initialises a tracker using initial bounding box.
		"""
		# define constant velocity model
		# self.kf = KalmanFilter(dim_x=10, dim_z=7)       
		# self.kf.F = np.array([[1,0,0,0,0,0,0,1,0,0],      # state transition matrix
		#                       [0,1,0,0,0,0,0,0,1,0],
		#                       [0,0,1,0,0,0,0,0,0,1],
		#                       [0,0,0,1,0,0,0,0,0,0],  
		#                       [0,0,0,0,1,0,0,0,0,0],
		#                       [0,0,0,0,0,1,0,0,0,0],
		#                       [0,0,0,0,0,0,1,0,0,0],
		#                       [0,0,0,0,0,0,0,1,0,0],
		#                       [0,0,0,0,0,0,0,0,1,0],
		#                       [0,0,0,0,0,0,0,0,0,1]])     

		# self.kf.H = np.array([[1,0,0,0,0,0,0,0,0,0],      # measurement function,
		#                       [0,1,0,0,0,0,0,0,0,0],
		#                       [0,0,1,0,0,0,0,0,0,0],
		#                       [0,0,0,1,0,0,0,0,0,0],
		#                       [0,0,0,0,1,0,0,0,0,0],
		#                       [0,0,0,0,0,1,0,0,0,0],
		#                       [0,0,0,0,0,0,1,0,0,0]])

		# x, y, r, vx, vy
		self.kf = KalmanFilter(dim_x=5, dim_z=3)       
		self.kf.F = np.array([[1,0,0,1,0],      # state transition matrix
		                      [0,1,0,0,1],
		                      [0,0,1,0,0],
		                      [0,0,0,1,0],  
		                      [0,0,0,0,1]])     

		self.kf.H = np.array([[1,0,0,0,0],      # measurement function,
		                      [0,1,0,0,0],
		                      [0,0,1,0,0]])


		self.kf.R *= 1e-2
		
		# self.kf.R[0:,0:] *= 10.   # measurement uncertainty
		self.kf.P[3:, 3:] *= 1000. 	# state uncertainty, give high uncertainty to the unobservable initial velocities, covariance matrix
		self.kf.P *= 10.

		# self.kf.Q[-1,-1] *= 0.01    # process uncertainty
		self.kf.Q[3:, 3:] *= 5e-1
		self.kf.x[:3] = det2D.reshape((3, 1)) # x, y, r

		self.time_since_update = 0
		self.id = KalmanBoxTracker.count
		KalmanBoxTracker.count += 1
		self.history = []
		self.hits = 1           # number of total hits including the first detection
		self.hit_streak = 1     # number of continuing hit considering the first detection
		self.first_continuing_hit = 1
		self.still_first = True
		self.age = 0
		self.info = info        # other info associated

	def update(self, det2D, info): 
		""" 
		Updates the state vector with observed bbox.
		"""
		self.time_since_update = 0
		self.history = []
		self.hits += 1
		self.hit_streak += 1          # number of continuing hit
		if self.still_first:
			self.first_continuing_hit += 1      # number of continuing hit in the fist time

		self.kf.update(det2D)
		self.info = info

	def predict(self):       
		"""
		Advances the state vector and returns the predicted bounding box estimate.
		"""
		self.kf.predict()

		self.age += 1
		if (self.time_since_update > 0):
			self.hit_streak = 0
			self.still_first = False
		self.time_since_update += 1
		self.history.append(self.kf.x)
		return self.history[-1]

	def get_state(self):
		"""
		Returns the current bounding box estimate.
		"""
		# return self.kf.x[:7].reshape((7, ))
		# self.kf.x[3] = np.arctan2(self.kf.x[8], self.kf.x[7]) 
		return self.kf.x[:5].reshape((5, ))