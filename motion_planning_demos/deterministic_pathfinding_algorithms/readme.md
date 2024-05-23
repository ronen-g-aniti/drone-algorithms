# Deterministic Pathfinding Algorithms: Codes & Demonsrtations

## Table of Contents
1. [Introduction](#introduction)
2. [Some Conceptual Points](#concepts)
3. [Installation](#installation)
4. [Usage](#usage)

## Introduction <a class="anchor" id="introduction"></a>

This repository contains the implementation of the following deterministic pathfinding algorithms:
- Breadth-First Search (BFS)
- Depth-First Search (DFS)
- Uniform Cost Search (UCS)
- A* Search

The algorithms are implemented in Python and are demonstrated using the free space representation
constructed by the codes comprising the "free_space_construction" directory. 

Free space is modeled as a graph data structure in three dimensional space. Free space is structured as
a cubic lattice arrangement. Each node in the lattice represents a point in space. The edges of the graph
represent the connections between the nodes. Depending on the configuration parameters for the free space
construction, nodes are either connected to only their rectangular neighbors or connected to only their 
rectangular and diagonal neighbors.

## Some Conceptual Points <a class="anchor" id="concepts"></a>

### Breadth-First Search (BFS)
BFS is a blind search algorithm that explores the nodes in the order of their distance from the start node.
It's strengths for pathfinding applications are that it is guaranteed to find the shortest path between the
start and goal nodes and that it is complete. However, BFS is not efficient in terms of memory usage and
time complexity. It's not suitable for pathfinding applications where the cost of the edges is not uniform.

### Depth-First Search (DFS)
DFS is a blind search algorithm that explores the nodes in the order of their depth from the start node.
It's strengths for pathfinding applications are that it is memory efficient and that it is complete. However,
DFS is not guaranteed to find the shortest path between the start and goal nodes and it is not efficient in
terms of time complexity. Lots of the time, DFS will find a path that is not the shortest path.

### Uniform Cost Search (UCS)
UCS is a blind search algorithm that explores the nodes in the order of their cost from the start node.
It's strengths for pathfinding applications are that it is guaranteed to find the shortest path between the
start and goal nodes and that it is complete. However, UCS is not efficient in terms of memory usage and
time complexity. UCS does have the advantage of being able to handle non-uniform edge costs.

### A* Search
A* Search is an informed search algorithm that explores the nodes in the order of their cost from the start
node plus a heuristic estimate of the cost to the goal node. It's strengths for pathfinding applications are
that it is guaranteed to find the shortest path between the start and goal nodes and that it is complete. A*
Search is more efficient than BFS, DFS, and UCS in terms of memory usage and time complexity. A* Search is
suitable for pathfinding applications where the cost of the edges is not uniform. A* search is a widely used 
algorithm for pathfinding applications in the field of robotics.

## Installation <a class="anchor" id="installation"></a>

The following Python packages are required to run the codes in this repository:
- numpy
- matplotlib
- utm

You can install the required packages using the following command:

```bash
pip install numpy matplotlib utm
```

Once the required packages are installed, you can clone this repository using the following command:

```bash
git clone
```

## Usage <a class="anchor" id="usage"></a>

To run the demonstrations of the deterministic pathfinding algorithms, you can use the following commands:

```bash
cd deterministic_pathfinding_algorithms
python breadth_first_search.py
python depth_first_search.py
python uniform_cost_search.py
python a_star_search.py
```

Each of the Python scripts will generate a plot of the path found by the algorithm. 

Change the search configuration parameters in the Python scripts to see how the algorithms perform with
different configurations.