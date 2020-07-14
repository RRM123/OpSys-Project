import numpy as np

class Random(object):
	def __init__(self):
		self.n = 0

	def srand48(self, seed):
		self.n = (seed << 16) + 0x330e

	def next_random(self):
		self.n = (25214903917 * self.n + 11) & (2**48 - 1)
		return self.n

	def drand48(self):
		return float(self.next_random()) / (2**48)

	def get_random(self, lamb, upper_bound):
		rand_num = upper_bound
		while rand_num >= upper_bound:
			rand_num = -np.log(self.drand48()) / lamb
		return rand_num