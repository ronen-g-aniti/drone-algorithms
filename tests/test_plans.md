# Test Plans

## Test Degree of Being Shortest Distance

## Test Average Execution Time

## Test Average Path Distance from Obstacles

## Test Sparsity of Paths




# Test of RRT Path Performance
## Procudure

### Setup
1. Configure the drone's mission environment
1. Construct a free space lattice with default parameters for comparison purposes
1. Generate random start and goal points in free space
1. Run the RRT algorithm to find a path from start to goal
1. Return the path points 

### Analysis
1. Calculate the path distance
1. Measure the execution time to generate the path
1. Run an algorithm to compute the average distance of points along path from nearest obstacles
1. Run an algorithm to compute the sparsity of the path
1. Run a search using A* from the same start and end point and compare the path distances using a ratio

# Test of A* Path Performance
## Procudure