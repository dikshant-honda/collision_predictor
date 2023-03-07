# ! /usr/bin/env python3

import numpy as np 
from shapely import Point
import dubins

class LaneInfo:
	def __init__(self) -> None:
		# ---------------- defining all the lanes ----------------

		# straight lines information
		x1, y1, yaw1 = self.get_straight_dubins([13, -0.9], [2.5, -0.9], np.pi, np.pi)     		# down
		x2, y2, yaw2 = self.get_straight_dubins([2.5, 0.9], [13, 0.9], 0, 0)             		# down 

		x3, y3, yaw3 = self.get_straight_dubins([-2.5, -0.9], [-13, -0.9], np.pi, np.pi)   		# up
		x4, y4, yaw4 = self.get_straight_dubins([-13, 0.9], [-2.5, 0.9], 0, 0)      	  			# up

		x5, y5, yaw5 = self.get_straight_dubins([-0.9, -13], [-0.9, -2.5], np.pi/2, np.pi/2) 	# left
		x6, y6, yaw6 = self.get_straight_dubins([0.9, -2.5], [0.9, -13], 1.5*np.pi, 1.5*np.pi) 	# left

		x7, y7, yaw7 = self.get_straight_dubins([-0.9, 2.5], [-0.9, 13], np.pi/2, np.pi/2) 		# right
		x8, y8, yaw8 = self.get_straight_dubins([0.9, 13], [0.9, 2.5], 1.5*np.pi, 1.5*np.pi) 		# right

		# intersection information
		x9, y9, yaw9 = self.get_dubins([2.5, -0.9], [0.9, -2.5], np.pi, 1.5*np.pi)				# down to left
		x10, y10, yaw10 = self.get_dubins([2.5, -0.9], [-2.5, -0.9], np.pi, np.pi)				# down to up
		x11, y11, yaw11 = self.get_dubins([2.5, -0.9], [-0.9, 2.5], np.pi, np.pi/2)				# down to right

		x12, y12, yaw12 = self.get_dubins([-0.9, -2.5], [-2.5, -0.9], np.pi/2, np.pi)			# left to up
		x13, y13, yaw13 = self.get_dubins([-0.9, -2.5], [-0.9, 2.5], np.pi/2, np.pi/2)			# left to right
		x14, y14, yaw14 = self.get_dubins([-0.9, -2.5], [2.5, 0.9], np.pi/2, 0)					# left to down

		x15, y15, yaw15 = self.get_dubins([-2.5, 0.9], [-0.9, 2.5], 0, np.pi/2)					# up to right
		x16, y16, yaw16 = self.get_dubins([-2.5, 0.9], [2.5, 0.9], 0, 0)							# up to down
		x17, y17, yaw17 = self.get_dubins([-2.5, 0.9], [0.9, -2.5], 0, 1.5*np.pi)					# up to left

		x18, y18, yaw18 = self.get_dubins([0.9, 2.5], [2.5, 0.9], 1.5*np.pi, 0)					# right to down
		x19, y19, yaw19 = self.get_dubins([0.9, 2.5], [0.9, -2.5], 1.5*np.pi, 1.5*np.pi)			# right to left
		x20, y20, yaw20 = self.get_dubins([0.9, 2.5], [-2.5, -0.9], 1.5*np.pi, np.pi)				# right to up

		# lane information
		self.lane_1 = [self.arr_to_point(x1, y1), yaw1]
		self.lane_2 = [self.arr_to_point(x2, y2), yaw2]
		self.lane_3 = [self.arr_to_point(x3, y3), yaw3]
		self.lane_4 = [self.arr_to_point(x4, y4), yaw4]
		self.lane_5 = [self.arr_to_point(x5, y5), yaw5]
		self.lane_6 = [self.arr_to_point(x6, y6), yaw6]
		self.lane_7 = [self.arr_to_point(x7, y7), yaw7]
		self.lane_8 = [self.arr_to_point(x8, y8), yaw8]
		self.lane_9 = [self.arr_to_point(x9, y9), yaw9]
		self.lane_10 = [self.arr_to_point(x10, y10), yaw10]
		self.lane_11 = [self.arr_to_point(x11, y11), yaw11]
		self.lane_12 = [self.arr_to_point(x12, y12), yaw12]
		self.lane_13 = [self.arr_to_point(x13, y13), yaw13]
		self.lane_14 = [self.arr_to_point(x14, y14), yaw14]
		self.lane_15 = [self.arr_to_point(x15, y15), yaw15]
		self.lane_16 = [self.arr_to_point(x16, y16), yaw16]
		self.lane_17 = [self.arr_to_point(x17, y17), yaw17]
		self.lane_18 = [self.arr_to_point(x18, y18), yaw18]
		self.lane_19 = [self.arr_to_point(x19, y19), yaw19]
		self.lane_20 = [self.arr_to_point(x20, y20), yaw20]
	
	def get_dubins(self, start, end, theta0, theta1, step_size = 0.01):
		q0 = (start[0], start[1], theta0)
		q1 = (end[0], end[1], theta1)

		turning_radius = 1.55
		path = dubins.shortest_path(q0, q1, turning_radius)
		
		configurations, _ = path.sample_many(step_size)

		x, y, yaw = np.array([]), np.array([]), np.array([])
		for i in range(len(configurations)):
			x = np.append(x, configurations[i][0])
			y = np.append(y, configurations[i][1])
			yaw = np.append(yaw, configurations[i][2])
		
		return x, y, yaw

	def get_straight_dubins(self, start, end, theta0, theta1, step_size = 0.01):
		q0 = (start[0], start[1], theta0)
		q1 = (end[0], end[1], theta1)

		turning_radius = 0.0001

		path = dubins.shortest_path(q0, q1, turning_radius)
		configurations, _ = path.sample_many(step_size)

		x, y, yaw = np.array([]), np.array([]), np.array([])
		for i in range(len(configurations)):
			x = np.append(x, configurations[i][0])
			y = np.append(y, configurations[i][1])
			yaw = np.append(yaw, configurations[i][2])

		return x, y, yaw

	def arr_to_point(self, x, y):
		point_arr = []
		for i in range(len(x)):
			point_arr.append(Point(x[i], y[i], 0))
		return point_arr

	def point_to_arr(self, lane):
		arr_x, arr_y = [], []
		for i in range(len(lane[0])):
			arr_x.append(lane[0][i].x)
			arr_y.append(lane[0][i].y)
		return arr_x, arr_y
	
	def stack_lanes(self, prev_lane, next_lane):
		if len(next_lane) == 0:
			return prev_lane
		prev_arr_x, prev_arr_y = self.point_to_arr(prev_lane)
		next_arr_x, next_arr_y = self.point_to_arr(next_lane)
		lane_x = np.hstack((prev_arr_x, next_arr_x))
		lane_y = np.hstack((prev_arr_y, next_arr_y))
		lane = [self.arr_to_point(lane_x, lane_y), np.hstack((prev_lane[1], next_lane[1]))]
		return lane

	def get_turning_routes(self, original_lane):
		if original_lane == self.lane_1:
			return [self.lane_9, self.lane_10, self.lane_11]
		if original_lane == self.lane_5:
			return [self.lane_12, self.lane_13, self.lane_14]
		if original_lane == self.lane_4:
			return [self.lane_15, self.lane_16, self.lane_17]
		if original_lane == self.lane_8:
			return [self.lane_18, self.lane_19, self.lane_20]
		
	def get_linking_route(self, turning_route):
		if turning_route == self.lane_9:
			merging_route = self.lane_6
		if turning_route == self.lane_10:
			merging_route = self.lane_3
		if turning_route == self.lane_11:
			merging_route = self.lane_7
		if turning_route == self.lane_12:
			merging_route = self.lane_3
		if turning_route == self.lane_13:
			merging_route = self.lane_7
		if turning_route == self.lane_14:
			merging_route = self.lane_2
		if turning_route == self.lane_15:
			merging_route = self.lane_7
		if turning_route == self.lane_16:
			merging_route = self.lane_2
		if turning_route == self.lane_17:
			merging_route = self.lane_6
		if turning_route == self.lane_18:
			merging_route = self.lane_2
		if turning_route == self.lane_19:
			merging_route = self.lane_6
		if turning_route == self.lane_20:
			merging_route = self.lane_3
		return merging_route

if __name__ == '__main__':
	# defining the lane info object
	lanes = LaneInfo()