# Stochastic Path Planning: Codes & Demonstrations

## Table of Contents
1. [Introduction](#introduction)
1. [Main Concepts](#main-concepts)
1. [Installation](#installation)
1. [Usage](#usage)

## Introduction <a class="anchor" id="introduction"></a>

This repository contains the implementation of the following stochastic path planning algorithms:
- Probabilistic Roadmap (PRM)
- Rapidly exploring Random Tree (RRT)

This project consists of two main files: "prm.py" and "rrt.py". The first file contains the implementation of the PRM algorithm, while the second file contains the implementation of the RRT algorithm. Both files contain a main function that demonstrates the usage of the algorithms.

## Main Concepts <a class="anchor" id="main-concepts"></a>

### Probabilistic Roadmap (PRM)
The Probabilistic Roadmap (PRM) algorithm is a sampling-based path planning algorithm that constructs a roadmap of the configuration space. The roadmap is a graph where the nodes represent the collision-free configurations of the robot and the edges represent the collision-free paths between the configurations. The PRM algorithm consists of the following steps:
1. Sample the configuration space to generate a set of random configurations.
1. Connect the configurations to form a roadmap.
1. Find a path between the start and goal configurations using the roadmap.

### Rapidly exploring Random Tree (RRT)
The Rapidly exploring Random Tree (RRT) algorithm is a sampling-based path planning algorithm that constructs a tree of the configuration space. The tree is a graph where the nodes represent the collision-free configurations of the robot and the edges represent the collision-free paths between the configurations. The RRT algorithm consists of the following steps:
1. Sample a random configuration in the configuration space.
1. Extend the tree towards the random configuration.
1. Repeat the above steps until the goal configuration is reached.

#### RRT Details
In my implementation of the Rapidly-exploring Random Tree (RRT) algorithm, the steering function plays a critical role by dynamically guiding the robot from its current state towards a target configuration within the constraints of fixed speed, maximum steering angle rate, and Euler integration for path development. This function systematically biases the sampling process towards the goal and employs incremental motion integration, ensuring that the path planning respects both collision avoidance and boundary constraints, crucial for navigating complex 3D environments.

The mathematical model for the steering function is as follows:

$$
\begin{aligned}
& \text{Given the current state } (\mathbf{x}_i, \mathbf{q}_i) \text{ and target position } \mathbf{x}_{\text{target}}, \text{ where } \mathbf{x}_i, \mathbf{q}_i \in \mathbb{R}^3: \\
& \textbf{1. Compute Direction:} \\
& \quad \mathbf{u} = \frac{\mathbf{x}_{\text{target}} - \mathbf{x}_i}{\|\mathbf{x}_{\text{target}} - \mathbf{x}_i\|} \\
& \textbf{2. Determine Rotation Axis and Angle:} \\
& \quad \mathbf{k} = \frac{\mathbf{q}_i \times \mathbf{u}}{\|\mathbf{q}_i \times \mathbf{u}\|} \\
& \quad \theta = \arccos\left(\frac{\mathbf{q}_i \cdot \mathbf{u}}{\|\mathbf{q}_i\| \|\mathbf{u}\|}\right) \\
& \quad \theta = \min(\theta, \Delta \theta_{\text{max}}) \\
& \textbf{3. Update Orientation using Rodrigues' Rotation Formula:} \\
& \quad \mathbf{q}_{\text{new}} = \cos(\theta)\mathbf{q}_i + \sin(\theta)(\mathbf{k} \times \mathbf{q}_i) + (1 - \cos(\theta))\mathbf{k}(\mathbf{k} \cdot \mathbf{q}_i) \\
& \textbf{4. Update Position using Euler Integration:} \\
& \quad \mathbf{x}_{\text{new}} = \mathbf{x}_i + \mathbf{q}_{\text{new}} \cdot v \cdot \Delta t
\end{aligned}
$$

Here, $\Delta t$ denotes the time step for Euler integration, and $v$ represents the constant velocity. The RRT algorithm iteratively applies this steering model to expand the tree from the start state towards the goal, effectively drawing a path that is constrained by the robot's dynamics and environmental factors. This ensures that each new node in the tree is reachable and adheres to the physical capabilities and limits of the robot, fostering a feasible and optimal pathfinding strategy.


## Installation <a class="anchor" id="installation"></a>
To run the code, you need to have Python installed on your system. You can download Python from the official website: https://www.python.org/downloads/. Then , you can clone this repository using the following command:

```bash
git clone 
```

For this project, you will need to have the following Python libraries installed:
- numpy
- matplotlib
- SciPy
- UTM


## Usage <a class="anchor" id="usage"></a>
To run the PRM algorithm, you can use the following command:

```bash
python prm.py
```

To run the RRT algorithm, you can use the following command:

```bash
python rrt.py
```


Both files contain a main function that demonstrates the usage of the algorithms. The main function a map representation based on the obstacle data from "map_data/colliders.csv", start and goal configurations, and then runs the PRM or RRT algorithm to find a path between the start and goal configurations. The path is visualized using the matplotlib library.


