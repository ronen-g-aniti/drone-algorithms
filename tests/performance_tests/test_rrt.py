import sys
import os

# Add the project root directory to the sys.path list
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

# Now import the RRT module correctly
from motion_planning_demos.stochastic_pathfinding_algorithms.rrt_with_steering import RRT
from motion_planning_demos.free_space_construction.environment_data import EnvironmentData

import numpy as np
import time

def generate_random_gps(x_bounds, y_bounds, z_bounds):
    """
    Generate a random GPS coordinate within the specified bounds.
    """
    x = np.random.uniform(x_bounds[0], x_bounds[1])
    y = np.random.uniform(y_bounds[0], y_bounds[1])
    z = np.random.uniform(z_bounds[0], z_bounds[1])
    return [x, y, z]

def test_rrt_performance(num_tests=10, environment_path='../motion_planning_demos/free_space_construction/map_data/colliders.csv',
                         x_bounds=(-122.398, -122.395), y_bounds=(37.790, 37.794), z_bounds=(50, 200)):
    times = []
    env_obj = EnvironmentData(environment_path, 5.0)
    
    for _ in range(num_tests):
        start_gps = generate_random_gps(x_bounds, y_bounds, z_bounds)
        goal_gps = generate_random_gps(x_bounds, y_bounds, z_bounds)
        
        try:
            rrt = RRT(env_obj, start_gps, goal_gps, GOAL_BIAS=0.7, MAX_STEER_ANGLE_RATE=np.pi/24, TIME_STEP=0.1, 
                      TIME_INTERVAL=5.0, SPEED=2.0, MAX_ITERATIONS=10000, GOAL_TOLERANCE=1.0)
        except ValueError as e:
            print(f"Invalid start or goal state: {e}")
            continue

        start_time = time.time()
        try:
            path = rrt.run()
            end_time = time.time()

            if path is not None:
                times.append(end_time - start_time)
            else:
                print(f"No path found for start: {start_gps}, goal: {goal_gps}")
        except Exception as e:
            print(f"Error during RRT run: {e}")

    if times:
        average_time = sum(times) / len(times)
        print(f"Average time to find a path: {average_time:.4f} seconds over {num_tests} tests")
    else:
        print("No paths were found in any of the tests")

if __name__ == '__main__':
    # Add the project root directory to the sys.path list
    project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
    if project_root not in sys.path:
        sys.path.insert(0, project_root)

    test_rrt_performance()