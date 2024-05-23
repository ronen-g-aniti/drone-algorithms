import numpy as np

"""
This file contains functions for checking collisions between a point and an environment.
The reason for the different collision check functions is to compare the performance of different methods.
"""

def collision_check_basic(environment_data_object, point):
    """
    Check if a point is in collision with the environment.
    
    Args:
        environment_data_object (EnvironmentData): an object of the EnvironmentData class
        point (numpy.ndarray): a numpy array of shape (3,) containing the point to check
        
    Returns:
        bool: a boolean indicating whether the point is in collision with the environment
    
    Note:
        This function uses a `for` loop to iterate over the obstacles in the environment and check if the point is in collision with any of them.
        The benefits of this function are that it is simple to implement and understand.
        The drawbacks are that it is not vectorized and may be slow for large environments with many obstacles.
    """
    for idx, obstacle_center in enumerate(environment_data_object.centers):
        if abs(point[0] - obstacle_center[0]) <= environment_data_object.halfsizes[idx, 0]:
            if abs(point[1] - obstacle_center[1]) <= environment_data_object.halfsizes[idx, 1]:
                if abs(point[1] - obstacle_center[2]) <= environment_data_object.halfsizes[idx, 2]:
                    return True
    return False

def collision_check_vectorized(environment_data_object, point):
    """
    Check if a point is in collision with the environment.    
    
    Args:
        environment_data_object (EnvironmentData): an object of the EnvironmentData class
        point (numpy.ndarray): a numpy array of shape (3,) containing the point to check
        
    Returns:
        bool: a boolean indicating whether the point is in collision with the environment
        
    Note:
        This function uses vectorized operations to check if the point is in collision with any of the obstacles in the environment.
        The benefits of this function are that it is more efficient than the basic collision check function and is easy to understand.
        The drawbacks are that it may be slower than the KDTree collision check function for some environments.
    """
    
    broadcasted_point = np.tile(point, (len(environment_data_object.centers),1))
    deltas = np.abs(broadcasted_point - environment_data_object.centers)
    collision_conditions = (deltas <= environment_data_object.halfsizes)
    return np.any(np.all(collision_conditions, axis=1))

def collision_check_spatial(environment_data_object, point):
    """
    Check if a point is in collision with the environment.
    
    Args:
        environment_data_object (EnvironmentData): an object of the EnvironmentData class
        point (numpy.ndarray): a numpy array of shape (3,) containing the point to check
        
    Returns: 
        bool: a boolean indicating whether the point is in collision with the environment
    
    Note: 
        This function uses a KDTree to query the nearest obstacles to the point and check if the point is in collision with any of them.
        The benefits of this function are that it is more efficient than the basic collision check function and is easy to understand.
        The drawbacks are that it may be slower than the vectorized collision check function for some environments.
        Also, the KDTree query may return false positives if the point is close to an obstacle but not in collision with it.
        Furthermore, the number of nearest obstacles to query is hardcoded to 4, which may not be optimal for all environments.
    """
    _, indices = environment_data_object.ground_centers.query([point[:2]], k=4)
    for i in indices[0]:
        if environment_data_object.heights[i] >= point[2]:
            if abs(environment_data_object.centers[i][0] - point[0]) <= environment_data_object.halfsizes[i][0]:
                if abs(environment_data_object.centers[i][1] - point[1]) <= environment_data_object.halfsizes[i][1]:
                    return True
    return False

def inside_environment(environment_data_object, point):
    """
    Check if a point is inside the environment.
    
    Args:
        environment_data_object (EnvironmentData): an object of the EnvironmentData class
        point (numpy.ndarray): a numpy array of shape (3,) containing the point to check
        
    Returns:
        bool: a boolean indicating whether the point is inside the environment
    """
    if point[0] < environment_data_object.xbounds[0] or point[0] > environment_data_object.xbounds[1]:
        return False
    if point[1] < environment_data_object.ybounds[0] or point[1] > environment_data_object.ybounds[1]:
        return False
    if point[2] < environment_data_object.zbounds[0] or point[2] > environment_data_object.zbounds[1]:
        return False
    return True

def collision_check_two_points(environment_data_object, point1, point2, SPACING=1.0):
    """
    Check if the line segment between two points is in collision with the environment.
    
    Args:
        environment_data_object (EnvironmentData): an object of the EnvironmentData class
        point1 (numpy.ndarray): a numpy array of shape (3,) containing the start point of the line segment
        point2 (numpy.ndarray): a numpy array of shape (3,) containing the end point of the line segment
        SPACING (float): the spacing between test points on the line segment
        
    Returns:
        bool: a boolean indicating whether the line segment is in collision with the environment
    
    Note:
        The way this function works is by dividing the line segment into a number of test points based on the spacing parameter.
        It then checks if any of the test points are in collision with the environment.
        The benefits of this function are that it is simple to implement and understand.
        The drawbacks are that it may be slow for large line segments or small spacing values.
    """
    delta = point2 - point1
    distance = np.linalg.norm(delta)
    direction = delta / distance
    number_of_points = int(distance / SPACING)
    test_points = np.array([point1 + i * SPACING * direction for i in range(number_of_points + 1)])
    segment_evaluation = np.array([collision_check_vectorized(environment_data_object, test_point) for test_point in test_points])
    return np.any(segment_evaluation)
