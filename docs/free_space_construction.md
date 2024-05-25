# Free Space Construction

## Main Concept
For discrete path planning algorithms that require graphs to conduct search, it becomes necessary to create a graph representation of free space. The way I do that in this project is simple. It involves creating a grid of points. If any points is in an obstacle then I delete it. Then I connect all neighboring points that don't collide. 

## Typical Results