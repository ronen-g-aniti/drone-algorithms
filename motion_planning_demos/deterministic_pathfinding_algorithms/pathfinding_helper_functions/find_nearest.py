def find_nearest(free_space_lattice_object, query_pos):
	"""
	Find the nearest point in the free space lattice to the query position.
	
	Args:
		free_space_lattice_object (Lattice object): an object of the CubicLattice class
		query_pos (numpy.ndarray): a numpy array of shape (3,) containing the query position
		
	Returns: 
		int: the index of the nearest point in the free space lattice
	"""
	_, indices = free_space_lattice_object.free_space_points_kd_tree.query([query_pos], k=1)
	return indices[0]
