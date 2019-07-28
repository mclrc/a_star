## a_star
Basic A* Implementation in Python

### Usage
Import the module
```python
import a_star
```
```a_star.path``` is the main function of the algorithm.
```python
a_star.path(grid: List[List[a_star.Node]], start: Tuple[int, int], goal: Tuple[int, int], diagonals: bool, heuristic: callable -> a_star.Node
```
It takes four arguments:

| Argument | Type | Use |
| --- | --- | --- |
| grid | List[List[a_star.Node]] | The grid to find the path in |
| start | Tuple[int, int] | Coordinates of the starting point |
| goal | Tuple[int, int] | Coordinates of the goal |
| diagonals | bool | Allows diagonal movement if True. Default is True |
| heuristic | callable | Heuristic function to be used. Default is a_star.h_diagonal |

The return value is the target node. The nodes in the path are chained together backwards through their parent property.

### Included Heuristics
- ```a_star.h_dijkstra```: Turns A* into Dijkstras algorithm by disregarding H cost. Slow, but guarantees a perfect result
- ```a_star.h_manhattan```: Optimal for scenarios without diagonal movement
- ```a_star.h_diagonal```: Optimal for scenarios with diagonal movement

A basic grid can be generated using a_star.create_grid
```python
a_star.create_grid(width: int, height: int, obstacles: List[Tuple[int, int]]) -> List[List[a_star.Node]]
```
This will return a 2D array, ready to be passed to path. With unwalkable nodes at the positions in obstacles.

### Nodes
Nodes have a few properties you should be interested in

| Property | Type | Use |
| --- | --- | --- |
| cost | int | The G cost of walking over the node |
| walkable | bool | Specifies if the node can be walked over at all |

Their constructor looks like this:
```python
a_star.Node.__init__(self, x: int, y: int, walkable: bool = True)
```


### Example

```python
from a_star import *

# Create list of obstacle coordinates
obstacles = []
for i in range(0, 17):
	obstacles.append((14, i))
for i in range(3, 14):
	obstacles.append((i, 12))
  
# Create 2D array
example_map = create_grid(20, 20, obstacles)

# Run the algorithm
target = path(grid=example_map, start=(9, 0), goal=(19, 19), heuristic=h_diagonal)
  
# Output result

# Map 2D array of Nodes to 2D Array of strings to be output
v_grid = list(
	map(lambda c: list(map(lambda n: " " if n.walkable else "#", c)), example_map))

# Put "X"s along the path in the array
current = target
while not (current == example_map[9][0]):
	v_grid[current.x][current.y] = "X"
	current = current.parent

# Put an S at the starting point, and a G at the goal
v_grid[9][0] = "S"
v_grid[19][19] = "G"
  
# Output the 2D array
for y in range(20):
	output = ""
	for x in range(20):
		output += v_grid[x][y]
	print(output)

```
