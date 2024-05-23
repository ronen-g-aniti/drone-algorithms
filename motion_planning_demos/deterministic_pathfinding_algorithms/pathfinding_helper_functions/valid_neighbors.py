def valid_neighbors(free_space_lattice_object, current_index):
	"""
	Return the valid neighbors of a point in the free space lattice.
	
	Args:
		free_space_lattice_object (Lattice object): an object of the CubicLattice class
        current_index (int): the index of the current point in the free space lattice
		
	Returns:
		list: a list of tuples containing the neighbor index and the distance to the neighbor
	"""
	
	neighbors = free_space_lattice_object.graph[current_index]
	neighbor_pairs = [(neighbor_index, distance) for neighbor_index, distance in neighbors.items()]
	return neighbor_pairs 
