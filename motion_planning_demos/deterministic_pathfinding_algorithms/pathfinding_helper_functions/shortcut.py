# Add the project root directory to the sys.path list (motion_planning_demos)
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..','..'))
if project_root not in sys.path:
    sys.path.insert(0, project_root)
	
import numpy as np

from free_space_construction.free_space_helpers.collision_check import collision_check_vectorized, collision_check_two_points

def shortcut(environment_data_object, input_path, points):
	"""	
	Removes unnecessary waypoints from the path while keeping the path collision-free.
	
	Args:
		environment_data_object (EnvironmentData): an object of the EnvironmentData class
        input_path (list): a list of integers representing the indices of the waypoints in the path
        points (numpy.ndarray): a numpy array of shape (n,3) containing the coordinates of the waypoints
		
	Returns:
		list: a list of integers representing the indices of the waypoints in the shorter path
		
	
	Note:
		The way the algorithm works is by checking if the line between two points is in collision with the environment. If it is not, the algorithm removes the intermediate points between the two points.
		The algorithm starts by checking the line between the first and last points in the path. If the line is in collision with the environment, the algorithm moves the last point one step closer to the first point and checks again.
	
	
	"""
	E = len(input_path) - 1
	S = 0

	e = E
	s = S

	P = [E]

	while s < e:
		shortcut_found = False
		while s < e:
			if collision_check_two_points(environment_data_object, points[input_path[s]],points[input_path[e]]):
				s += 1
			else:
				P.append(s)
				shortcut_found = True
				break
		if shortcut_found:
			e = P[-1]
			s = 0
		else:
			e -= 1
			P.append(e)
			s = 0


	P = P[::-1]
	shorter_path = [input_path[p] for p in P]
	

	return shorter_path

