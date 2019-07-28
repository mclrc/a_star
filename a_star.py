"""
A* implementation

Usage:
import A_star
last = A_star.path(grid, start, goal, heuristic)

grid: 2D array filled with Node objects. 
start: tuple(int, int) containing coordinates of the starting node in the grid
goal: tuple(int, int) containing coordinates of the target node in the grid
heuristic: heuristic function to be used to calculate a nodes H cost. Available are h_dijkstra, h_diagonal and h_manhattan

The return value is the target node. The nodes in the path are chained together backwards through their parent property

A basic grid can be generated using create_grid(width, height, array(tuple(int, int)))
This will return a 2D array with unwalkable nodes at the positions passed to obstacles,
ready to be passed to path.

Nodes have a cost property to make them more expensive to walk over, useful for implementing different
terrains, like mountains or rivers in a game.

"""
_start = None
_goal = None
_grid = [[]]
_open = []
_closed = []
_current = None
_D = 1
_allow_diagonals = True

# Diagonal distance with tiebreaker


def h_diagonal(n):
	dx = abs(n.x - _goal.x)
	dy = abs(n.y - _goal.y)
	return _D * (dx + dy) + (_D - 2 * _D) * min(dx, dy)

# Manhattan heuristic with tiebreaker, optimal if you have set _allow_diagonals to False
def h_manhattan(n):
	if not n.h == None:
		return n.h

	dx = abs(n.x - _goal.x)
	dy = abs(n.y - _goal.y)

	return _D * (dx + dy) * (1 + 1/100)

# This basically turns A* into Dijkstras aLgorithm. Very slow, but guarantees a perfect end result
def h_dijkstra(n):
	return 0

def _g(n):
	return (_g(n.parent) + n.cost if n.g == None else n.g)

def _f(n):
	if n.g == None or n.h == None:
		return 0
	return n.g + n.h

def _process(n, heuristic):
	# Remove node from open list and add to closed list
	_open.remove(n)
	_closed.append(n)
		# If the node is the goal, done. Return
	if n == _goal:
		return
	# Loop through the nodes neighbors in an attempt to find the next best step
	for node in n.get_neighbors():
		# If the node is not walkable, or we already processed it, move on to the next neighbor
		if not node.walkable or node in _closed:
			continue
		# If the node is not in the open list, add it and qeue it for processing. Calculate cost and set parent to current node
		if not node in _open:
			_open.append(node)

			node.reset()
			node.parent = n
			node.g = _g(node)
			node.h = heuristic(node)
			node.f = node.g + node.h
		# If the node is in the open list, check if the current path is a better way to get to it. If so, recalculate
		else:
			if node.g < n.g - node.cost:
				node.parent = n
				node.g = None
				node.g = _g(node)

def path(grid,
	start=(0, 0),
	goal = (10, 10),
	diagonals = True,
	heuristic=h_diagonal
):
	global _grid, _start, _goal, _open, _closed, _current
	# Set values in preparation
	_grid = grid
	_start = grid[start[0]][start[1]]
	_goal = grid[goal[0]][goal[1]]
	_open = [_start]
	_closed = []
	_current = _start
	__allow_diagonals = diagonals

	_start.g, _start.f, _start.h = [0, 0, 0]

	while not _goal in _closed:
		# Find lowest F cost node in open list
		_current = sorted(_open, key=_f)[0]
		# Process current node
		_process(_current, heuristic)
		# Return last node in path, the goal
	return _goal

def create_grid(width, height, obstacles):
	g = [[0 for j in range(height)] for i in range(width)]
	for y in range(height):
		for x in range(width):
			g[x][y] = Node(x, y, True)
	
	for obstacle in obstacles:
		g[obstacle[0]][obstacle[1]].walkable = False
	
	return g
# Class to represent a single grid node for the algorithm
class Node:

	walkable = True
	neighbors = None
	f = None
	g = None
	h = None
	parent = None
	cost = 1

	def __init__(self, x, y, walkable=True):
		self.reset()
		self.x = x
		self.y = y
		self.walkable = walkable

	def get_neighbors(self):
		# If the array already exists, return it to prevent repetition
		if not self.neighbors == None:
			return self.neighbors

		self.neighbors = []

		on_left_edge = self.x == 0
		on_top_edge = self.y == 0
		on_right_edge = self.x == len(_grid) - 1
		on_bottom_edge = self.y == len(_grid[0]) - 1

		# Add every existing neighbor to the array by checking for grid edges before trying to do so
		if not on_left_edge:
			self.neighbors.append(_grid[self.x - 1][self.y])
		if not on_right_edge:
			self.neighbors.append(_grid[self.x + 1][self.y])

		if not on_top_edge:
			self.neighbors.append(_grid[self.x][self.y - 1])
		if not on_bottom_edge:
			self.neighbors.append(_grid[self.x][self.y + 1])

		if _allow_diagonals:
			if not on_left_edge and not on_top_edge:
				self.neighbors.append(_grid[self.x - 1][self.y - 1])
			if not on_left_edge and not on_bottom_edge:
				self.neighbors.append(_grid[self.x - 1][self.y + 1])

			if not on_right_edge and not on_top_edge:
				self.neighbors.append(_grid[self.x + 1][self.y - 1])
			if not on_right_edge and not on_bottom_edge:
				self.neighbors.append(_grid[self.x + 1][self.y + 1])

		return self.neighbors

	# Reset values a different starting point or goal would change. Neighbors and coordinates stay the same
	def reset(self):
		self.f = None
		self.g = None
		self.h = None
		self.parent = None