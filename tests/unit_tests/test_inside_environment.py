import unittest
import numpy as np
import sys
import os

# Correct path to include the project's root directory
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

from motion_planning_demos.free_space_construction.free_space_helpers.collision_check import inside_environment
from motion_planning_demos.free_space_construction.environment_data import EnvironmentData

class TestInsideEnvironment(unittest.TestCase):
    def setUp(self):
        obstacle_geometry_file = "test_colliders.csv"
        with open(obstacle_geometry_file, 'w') as f:
            f.write("""lat0 37.792480, lon0 -122.397450
                    posX, posY, posZ, halfSizeX, halfSizeY, halfSizeZ
                    -50, -50, 25, 10, 10, 10
                    50, 50, 25, 10, 10, 10
                    """)
            
        self.environment_data_object = EnvironmentData(obstacle_geometry_file, 5.0)
        
    def tearDown(self):
        os.remove("test_colliders.csv")
    
    def test_point_inside(self):
        point = np.array([0, 0, 25])
        self.assertTrue(inside_environment(self.environment_data_object, point))
        
    def test_point_on_x_boundary(self):
        point = np.array([-60, -50, 25])
        self.assertTrue(inside_environment(self.environment_data_object, point))
        
    def test_point_on_y_boundary(self):
        point = np.array([-50, -60, 25])
        self.assertTrue(inside_environment(self.environment_data_object, point))
        
    def test_point_on_z_boundary(self):
        point = np.array([-50, -50, 35])
        self.assertTrue(inside_environment(self.environment_data_object, point))
        
if __name__ == '__main__':
    unittest.main()
        


