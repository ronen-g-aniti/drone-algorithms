import sys
import os

# Add the project root directory to the sys.path list (motion_planning_demos)
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

import numpy as np
from free_space_construction.environment_data import EnvironmentData
from free_space_construction.lattice import CubicLattice
from free_space_construction.free_space_helpers.reference_frame import global_to_local
from pathfinding_helper_functions.find_nearest import find_nearest
from pathfinding_helper_functions.valid_neighbors import valid_neighbors
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
 
def dfs(free_space_lattice_object, start_gps, goal_gps):
	"""
	Perform depth-first search (DFS) in the free space lattice to find a path from the start position to the goal position.
	
	Args:
		free_space_lattice_object (Lattice object): an object of the CubicLattice class
		start_gps (numpy.ndarray): a numpy array of shape (3,) containing the start GPS position
		goal_gps (numpy.ndarray): a numpy array of shape (3,) containing the goal GPS position
		
	Returns:
		list: a list of indices representing the path from the start position to the goal position
		
		
	Note:
		The DFS algorithm uses a LIFO queue to explore the lattice space. The algorithm continues to explore the lattice space until the goal position is found or all points have been visited.
        The benefits of this approach for pathfinding are that it is memory-efficient and can be faster than BFS for some problems.
        On the other hand, the DFS algorithm is not guaranteed to find the shortest path and can get stuck in infinite loops if the lattice is infinite.
	"""

	start_pos = global_to_local(start_gps, free_space_lattice_object.gps_home)
	start_index = find_nearest(free_space_lattice_object, start_pos)
	goal_pos = global_to_local(goal_gps, free_space_lattice_object.gps_home)
	goal_index = find_nearest(free_space_lattice_object, goal_pos)

	goal_found = False
	branch = {}
	branch[start_index] = None
	visited = set()

	lifo_queue = []
	lifo_queue.append(start_index)

	while lifo_queue:
		current_index = lifo_queue.pop()
		visited.add(current_index)
		if current_index == goal_index:
			print("Goal found")
			goal_found = True
			break
		for neighbor_index, _ in valid_neighbors(free_space_lattice_object, current_index):
			if neighbor_index not in visited:
				lifo_queue.append(neighbor_index)
				branch[neighbor_index] = current_index
	
	if goal_found:
		path = []
		while current_index != start_index:
			path.append(current_index)
			current_index = branch[current_index]
		path.append(start_index)
		path = path[::-1]

	return path

if __name__== "__main__":
	
	# Construct a free space lattice with a safety margin of 5.0 m
	env_data = EnvironmentData("../free_space_construction/map_data/colliders.csv", 5.0)
	
	# Define the start and goal GPS positions
	start_gps = np.array([-122.397450, 37.792480, 0.0])
	goal_gps = np.array([-122.397230, 37.792895, 50.0])
	
	# Create a free space lattice object
	center = np.array([0, 0, 50])
	halfsizes = np.array([50, 50, 50])
	lattice_obj = CubicLattice(env_data, center, halfsizes, resolution=15.0, connectivity="partial")

	# Perform depth-first search to find a path from the start to the goal position
	path = dfs(lattice_obj, start_gps, goal_gps)
	
	# Visualize the path in the free space lattice
	lattice_obj.visualize(env_data, path=path)
	
	# Learning Note:
	# Through visualizing paths found with DFS, it should eventually become clear
	# that the algorithm is not guaranteed to find the shortest path.