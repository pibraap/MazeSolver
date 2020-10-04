import cv2
import numpy as np
import imutils
import math
import sys


class PathFinder:
	def __init__(self, entry_point, exit_point, wall):
		self.grid_dim = (SIZE_MAZE, SIZE_MAZE)
		self.grid = np.full((self.grid_dim[0], self.grid_dim[1], 3), self.grid_dim[0]*self.grid_dim[1]*2+1)
		self.start_pos = (entry_point[1], entry_point[0])
		self.end_pos = (exit_point[1], exit_point[0])
		self.wall = wall
		self.solution = []



	def solve(self):
		LOADING = ['|', '/', '-', '\\']
		count = 0
		actual_pos = self.start_pos
		children = {}
		tested = []


		while actual_pos != self.end_pos:
			self.grid[actual_pos + (2,)] = self.grid_dim[0]*self.grid_dim[1]*2+1
			tested.append(actual_pos)


			new_pos = [(actual_pos[0], actual_pos[1]-1), (actual_pos[0]-1, actual_pos[1]), 
						(actual_pos[0], actual_pos[1]+1), (actual_pos[0]+1, actual_pos[1])]
			
			for pos in new_pos:
				if pos[0]<self.grid_dim[0] and pos[1]<self.grid_dim[1] and pos[0]>0 and pos[1]>0 and (self.wall[pos] != 255)  and (pos not in tested):
						
						dist_end = abs(pos[0] - self.end_pos[0]) + abs(pos[1] - self.end_pos[1])
						if actual_pos == self.start_pos:
							dist_start = 1
						else:
							dist_start = self.grid[actual_pos + (1,)] + 1

						self.grid[pos + (0,)] = dist_end
						self.grid[pos + (1,)] = dist_start
						self.grid[pos + (2,)] = dist_end + dist_start
							
						children[pos] = actual_pos
			if np.min(self.grid[:,:,2]) != self.grid_dim[0]*self.grid_dim[1]*2+1:
				try_pos = np.argwhere(self.grid[:,:,2] == np.min(self.grid[:,:,2]))
				min_dist_end = self.grid_dim[0]*self.grid_dim[1]*2+1
				for pos in try_pos:
					if self.grid[tuple(pos) + (0,)] < min_dist_end:
						min_dist_end = self.grid[tuple(pos) + (0,)]
						actual_pos =tuple(pos)


			else:
				self.solution = None
				break


		self.count = count
		if self.solution == []:
			back_pos = self.end_pos
			self.solution.append(back_pos)
			while back_pos != self.start_pos:

				back_pos = children[back_pos]
				self.solution.append(back_pos)

			self.solution.reverse()

		self.run = False



def find_angles(pos_ang):
	tol_tot = 0

	y1 = np.min(pos_ang[:,0])
	pos_first = (np.argwhere(pos_ang[:,0] == y1))
	tol_tot += len(list(pos_first))
	x1 = np.max(pos_ang[pos_first,1])
	ang1 = np.array([x1, y1])

	x2 = np.max(pos_ang[:,1])
	pos_first = (np.argwhere(pos_ang[:,1] == x2))
	tol_tot += len(pos_first)
	y2 = np.max(pos_ang[pos_first,0])
	ang2 = np.array([x2, y2])


	x3 = np.min(pos_ang[:,1])
	pos_first = (np.argwhere(pos_ang[:,1] == x3))
	tol_tot += len(pos_first)
	y3 = np.max(pos_ang[pos_first,0])
	ang3 = np.array([x3, y3])

	y4 = np.max(pos_ang[:,0])
	pos_first = (np.argwhere(pos_ang[:,0] == y4))
	tol_tot += len(pos_first)
	x4 = np.min(pos_ang[pos_first,1])
	ang4 = np.array([x4, y4])

	return np.array([ang1, ang2, ang3, ang4]), tol_tot

def rotate(pt, radians, origin):
    x, y = pt
    offset_x, offset_y = origin
    adjusted_x = (x - offset_x)
    adjusted_y = (y - offset_y)
    cos_rad = math.cos(radians)
    sin_rad = math.sin(radians)
    qx = offset_x + cos_rad * adjusted_x + sin_rad * adjusted_y
    qy = offset_y + -sin_rad * adjusted_x + cos_rad * adjusted_y

    return (int(qx), int(qy))

def is_a_maze(result):
	maze = False
	walls = [result[0,:], result[-1,:], result[:,0], result[:,-1]]
	shapes = [0, result.shape[0]-1, 0, result.shape[1]-1]
	points = []
	start_point = None
	end_point = None
	for i, wall in enumerate(walls):
		init_pix = 255
		change = []
		for j in range(len(wall)):
			if wall[j] != init_pix:
				change.append(j)
				init_pix = wall[j]
		if len(change) != 2 and len(change)!= 0:
			return maze, start_point, end_point
		else:
			if len(change) == 2:
				if i == 0 or i == 1:
					pos = (int((change[0]+change[1])/2), shapes[i])
					points.append(pos)
				else:
					pos = (shapes[i], int((change[0]+change[1])/2))
					points.append(pos)


	if len(points) != 2:
		return maze, start_point, end_point
	else:
		maze = True

		start_point = points[0]
		end_point = points[1]
		return maze, start_point, end_point

def original_point(corners, point):
	dist_point_0 = ((0-point[0])**2 + (0-point[1])**2)**(1/2) 
	dist_point_1 = ((SIZE_MAZE-point[0])**2 + (0-point[1])**2)**(1/2) 
	dist_point_2 = ((0-point[0])**2 + (SIZE_MAZE-point[1])**2)**(1/2) 
	dist_point_3 = ((SIZE_MAZE-point[0])**2 + (SIZE_MAZE-point[1])**2)**(1/2) 



	dist_point = min(dist_point_0, dist_point_1, dist_point_2, dist_point_3)

	if dist_point == dist_point_0:
		corner1 = corners[0]
		corner2 = corners[1]
		corner3 = corners[2]

	elif dist_point == dist_point_1:
		corner1 = corners[1]
		corner2 = corners[0]
		corner3 = corners[3]
		point = (SIZE_MAZE-point[0], point[1])

	elif dist_point == dist_point_2:
		corner1 = corners[2]
		corner2 = corners[3]
		corner3 = corners[0]
		point = (point[0], SIZE_MAZE-point[1])

	elif dist_point == dist_point_3:

		corner1 = corners[3]
		corner2 = corners[2]
		corner3 = corners[1]
		point = (SIZE_MAZE-point[0], SIZE_MAZE-point[1])



	dist_1_3 = ((corner1[0]-corner3[0])**2 + (corner1[1]-corner3[1])**2)**(1/2) 
	dist_1_2 = ((corner1[0]-corner2[0])**2 + (corner1[1]-corner2[1])**2)**(1/2)


	point = (point[0] * dist_1_2/SIZE_MAZE, point[1] * dist_1_3/SIZE_MAZE)

	transphorm_3 = ((corner3[0]-corner1[0])/dist_1_3, (corner3[1]-corner1[1])/dist_1_3)
	new_point_3 = (point[1] * transphorm_3[0], point[1]*transphorm_3[1]+10)

	transphorm_2 = ((corner2[0]-corner1[0])/dist_1_2, (corner2[1]-corner1[1])/dist_1_2)
	new_point_2 = (point[0] * transphorm_2[0] - 5, point[0]*transphorm_2[1] - 5)

	new_point = (int(corner1[0] + new_point_3[0] + new_point_2[0]), int(corner1[1] + new_point_3[1] + new_point_2[1]))

	return new_point




TOL_MAX = 30
SIZE_MAZE = 200
RATE_WALL = 0.0

LIGHT = np.array([80, 80, 80])
DARK = np.array([0, 0, 0]) 

# Delta from which the entry/exit points are considered to be the same.
DEL_SAME_POINT = 30

past_entry_point = (0, 0)
past_exit_point = (0, 0)

cap = cv2.VideoCapture('path/to/your/video.mov')
#cap = cv2.VideoCapture(0)

length_cap = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
ret, frame = cap.read()

solution = None
past_solution = None

font = cv2.FONT_HERSHEY_SIMPLEX

fourcc = cv2.VideoWriter_fourcc(*'MJPG')
out = cv2.VideoWriter('solution.mov', fourcc, 24, (int(frame.shape[1]/2), int(frame.shape[0]/2)))

count = 0
while cap.isOpened:
	count += 1
	sys.stdout.write('\r loading ' + str((np.round(count*100/length_cap, 1))) + ' %')
	sys.stdout.flush()    
	
	ret, frame = cap.read()
	mask = cv2.inRange(frame, DARK, LIGHT)

	pos_ang = np.argwhere(mask == 255)
	maze = False

	if pos_ang.size != 0 :
		corners, tol_tot = find_angles(pos_ang)

		rot = 0
		if tol_tot > TOL_MAX:
			rot = 20

			while tol_tot > TOL_MAX:
				rot -= 5
				mask_rot = imutils.rotate(mask, rot)
				pos_ang = np.argwhere(mask_rot == 255)
				corners, tol_tot = find_angles(pos_ang)

				if rot == 0:
					break


		h, w = mask.shape[:2]
		origin = (w/2, h/2)
		for i, corner in enumerate(corners):
			corner = rotate(corner, -rot*np.pi/180, origin)
			corners[i] = corner


		corner1 = corners[0]
		corner2 = corners[1]
		corner3 = corners[2]
		corner4 = corners[3]
		

		pts1 = np.float32([np.array(corner1), np.array(corner2), np.array(corner3), np.array(corner4)])
		pts2 = np.float32([[0, 0], [SIZE_MAZE, 0], [0, SIZE_MAZE], [SIZE_MAZE, SIZE_MAZE]])
		matrix = cv2.getPerspectiveTransform(pts1, pts2)
		result = cv2.warpPerspective(mask, matrix, (SIZE_MAZE, SIZE_MAZE))
		result[result>127] = 255
		result[result<=127] = 0

		
		kernel = np.ones((3, 3),np.uint8)
		result = cv2.dilate(result,kernel, iterations = 1)
		while result.sum()/(result.size*255) < RATE_WALL:
			result = cv2.dilate(result,kernel, iterations = 1)

		maze, entry_point, exit_point = is_a_maze(result)
		cv2.circle(result, entry_point, 1, (124) , -1)
		cv2.circle(result, exit_point, 1, (124) , -1)


		if maze:
			if (solution is None) or past_entry_point[0]-DEL_SAME_POINT > entry_point[0] or entry_point[0] > past_entry_point[0]+DEL_SAME_POINT \
			 or past_entry_point[1]-DEL_SAME_POINT > entry_point[1] or entry_point[1] > past_entry_point[1]+DEL_SAME_POINT \
			 or past_exit_point[0]-DEL_SAME_POINT > exit_point[0] or exit_point[0] > past_exit_point[0]+DEL_SAME_POINT \
			 or past_exit_point[1]-DEL_SAME_POINT > exit_point[1] or exit_point[1] > past_exit_point[1]+DEL_SAME_POINT:

				solution = PathFinder(entry_point, exit_point, result)
				solution.solve()
		
				ret, frame = cap.read()


			frame = cv2.putText(frame, 'path find', (50, 50), font, 2,(0,255,0), 5)

			past_entry_point = entry_point
			past_exit_point = exit_point
			entry_point = original_point(corners, entry_point)
			exit_point = original_point(corners, exit_point)

			old_pos = entry_point
			if solution.solution is None:
				solution.solution = past_solution
			else:
				for pos in solution.solution[::3]:
					pos = (pos[1], pos[0])
					pos = original_point(corners, pos)
					cv2.line(frame, pos, old_pos, (114,128,250), 5)
					old_pos = pos

			cv2.circle(frame, entry_point, 10, (255, 0, 0), -1)
			cv2.circle(frame, exit_point, 10, (255, 0, 0), -1)

			test_corners = [corners[0], corners[2], corners[3], corners[1], corners[0]]

			for i in range(len(test_corners)-1):
				cv2.line(frame, tuple(test_corners[i]), tuple(test_corners[i+1]), (0,255,0), 5)

			cv2.circle(frame, entry_point, 10, (0, 0, 255), -1)
			cv2.circle(frame, exit_point, 10, (0, 0, 255), -1)

			past_solution = solution.solution

	if not maze:
		frame = cv2.putText(frame, 'no maze detected', (50, 50), font, 2,(0,0,255), 5)



	out.write(cv2.resize(frame, (int(frame.shape[1]/2), int(frame.shape[0]/2))))






out.release()

