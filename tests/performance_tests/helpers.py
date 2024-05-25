import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial import KDTree
import sys
import os

# Add the project root directory to the sys.path list (motion_planning_demos) ## Change this? 
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

from motion_planning_demos.free_space_construction.environment_data import EnvironmentData
from motion_planning_demos.free_space_construction.lattice import CubicLattice
from motion_planning_demos.free_space_construction.free_space_helpers.reference_frame import global_to_local
from motion_planning_demos.deterministic_pathfinding_algorithms.pathfinding_helper_functions.find_nearest import find_nearest
from motion_planning_demos.deterministic_pathfinding_algorithms.pathfinding_helper_functions.valid_neighbors import valid_neighbors


def random_free_space_point_as_gps(environment_data_object):
    """
    Generate a random free space point within the environment bounds. Return as GPS coordinates.
    
    Args:
        environment_data_object (EnvironmentData): an object of the EnvironmentData class
        
    Returns:
        numpy.ndarray: a numpy array of shape (3,) containing the random free space point as GPS coordinates (latitude, longitude, altitude))
    """
    
def average_distance_from_obstacles(path, environment_data_object):
    """
    Calculate the average distance from the path to the obstacles in the environment. 
    Works by stepping along the path and measuring the distance to the nearest obstacle at each step.
    
    Args:
        path (list): a list of indices representing the path
        environment_data_object (EnvironmentData): an object of the EnvironmentData class
        
    Returns:
        float: the average distance from the path to the obstacles in the environment
    """)


if __name__ == "__main__":
    
    # Load the environment data
    environment_data_object = EnvironmentData("obstacle_geometry.csv", 5.0)

    # Create a cubic lattice
    free_space_lattice_object = CubicLattice(environment_data_object, 1.0)
   
    # Test Loop 1: RRT Performance Testing

    # Define the start and goal GPS coordinates
    start_gps = random_free_space_point_as_gps(environment_data_object)
    goal_gps = random_free_space_point_as_gps(environment_data_object)
    
    # Find the path, measuring the time to complete
    
    # Measure the path length
    
    # Measure the average distance from the path to the obstacles
    
    # Measure the path sparsity
    
    # Compute the same start-goal search using A* on a CubicLattice of default parameters
    
    # Return the ratio (L - L0) / L0
    

    
  

    
    
   