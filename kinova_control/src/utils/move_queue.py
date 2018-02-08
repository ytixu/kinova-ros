#! /usr/bin/env python

import json
import time
import copy


OUT_DIR = '/home/ytixu/ROS_pack/forked_arm/data/arm_poses/'
RAND_ID = time.time()

class MoveQueue:

	def __init__(self):
		self.queue = {}
		self.count = 0
		self.t = 0

	def push(self, r_pose, h_pose):
		self.queue[self.count] = {
			'r':r_pose,
			'h':h_pose}
		self.count += 1
		print self.count, ' pushed ~~~~~~~'
		return self.count

	def reset(self):
		self.queue = {}
		self.count = 0
		self.t += 1

	def record(self, temp=False):
		filename = 'temp.json'
		move_queue = copy.copy(self.queue)
		n = self.count
		t = self.t
		if not temp:
			self.reset()
			filename = 'kp-%d-%d.json' % (RAND_ID, t)
		if n > 5:
			with open(OUT_DIR+filename, 'w') as outfile:
			    json.dump({'poses_seq': move_queue, 'count': n, 'id':t}, outfile)
