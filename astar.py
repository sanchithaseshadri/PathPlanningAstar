import heapq as hq
from buildMap import Map
from util import radians
from safegoto import SafeGoTo
import copy

ROBOT_SIZE = 0.2 	# change this 
G_MULTIPLIER = 0.2
MOVES = [ (0.1, radians(0)), 	# move ahead
		  (-0.1, radians(0)), 	# move backwards
		  (0, radians(90)), 		# turn left
		  (0, -radians(90)) ]	# turn right
TOLERANCE = 1.0

class PathPlanner:
	def __init__(self, start, goal):
		# odometry - to get current location 
		self.odom_subscriber = rospy.Subscriber('/r1/odom', Odometry, self.odom_callback)
		self.pos = Pose()
		self.theta = 0

		# map remains constant
		self.map = Map()
		self.start = start
		self.goal = goal

	def odom_callback(self, odom):
		"""
		Callback function for the odometry subscriber, which will continuously update the 
		current position of the robot.
		@arg 	odom 	the odometry data of the robot
		"""		
		self.pos = odom.pose.pose
		self.theta = 2*math.atan2(self.pos.orientation.z, self.pos.orientation.w)

	def plan(self):
		final = a_star(self.start, self.goal, self.moves, self.map)
		if final == None:
			rospy.loginfo("Path not found.")
		else:
			rospy.loginfo("Constructing path..")
			path = self.construct_path(final)	# path in world coordinates
			# publish this path - goto for each of the path components? with a skip?
			# if using goto, convert each to world coordinates
			robot = SafeGoTo()
			robot.travel(path)


	def construct_path(self, end):
		"""
		backtrack from end to construct path
		"""
		current = end
		path = []	# path needs to be in world coordinates
		while current != None:
			path.append(current)
			current = current.parent
		return path


def a_star(start, end, map):
	# start, end are in world coordinates
	opened = []
	closed=[]
	final = None
	hq.heappush(opened, (0.0, start))

	while (final == None) and opened:
		# q is a Node object with x, y, theta
		q = heapq.heappop(opened)[1]
		for move in MOVES:		# move is in world coordinates

			if (q.is_move_valid(map, move)):
				next_node = q.apply_move(move)	# Node is returned in world coordinates
			else:
				next_node = None

			if next_node != None:
				if next_node.euclidean_distance(end) < TOLERANCE:
					final = next_node
					break
				# update heuristics h(n) and g(n)
				next_node.h = next_node.euclidean_distance(end)
				next_node.g = q.g + next_node.euclidean_distance(q)
				# f(n) = h(n) + g(n)
				next_node.f = G_MULTIPLIER * next_node.g + next_node.h
				next_node.parent = q

				# other candidate locations to put in the heap
				potential_open = any(other_f <= next_node.f and other_next.is_similar(next_node) for other_f, other_next in opened)
				
				if not potential_open:
					potential_closed = any(other_next.is_similar(next_node) and other_next.f <= next_node.f for other_next in closed)
					if not potential_closed:
						hq.heappush(opened, (next_node.f, next_node))
		closed.append(q)	

	return final				







