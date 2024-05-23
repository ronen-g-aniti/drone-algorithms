import numpy as np

def euclidean_distance(lattice_object, lattice_index_1, lattice_index_2):
	"""
	Calculate the Euclidean distance between two points in the free space lattice.
	
	Args:
		lattice_object (Lattice object): an object of the CubicLattice class
        lattice_index_1 (int): the index of the first point in the free space lattice
        lattice_index_2 (int): the index of the second point in the free space lattice
		
	Returns:
		float: the Euclidean distance between the two points
	"""
	return np.linalg.norm(lattice_object.free_space_points[lattice_index_1] - lattice_object.free_space_points[lattice_index_2])
