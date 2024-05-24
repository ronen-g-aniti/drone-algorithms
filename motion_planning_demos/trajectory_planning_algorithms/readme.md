# Trajectory Planning Code & Demonstration
This repository contains Python code for generating minimum snap trajectories in 3D space using seventh-order polynomial interpolation. The code demonstrates the process of planning smooth trajectories through a series of waypoints with a specified average speed.

## Table of Contents
- [Table of Contents](#table-of-contents)
- [Project Purpose](#project-purpose)
- [Mathematical Concepts](#mathematical-concepts)
- [Pre-requisites](#pre-requisites)
- [Usage](#usage)

## Project Purpose
The aim of this project is to provide a practical example of trajectory planning using polynomial interpolation techniques. This code can serve as a foundation for further exploration and development of trajectory planning algorithms for various applications, such as robotics, autonomous vehicles, and animation.

##  Mathematical Concepts
The core concept employed here is polynomial interpolation. For a trajectory with N waypoints, we construct N-1 polynomial segments, each represented by a seventh-order polynomial of the form:

$p(t) = a_7t^7 + a_6t^6 + a_5t^5 + a_4t^4 + a_3t^3 + a_2t^2 + a_1t + a_0$ where $a_i$ are the polynomial coefficients and $t$ is the normalized time within the segment.

To ensure smooth transitions between segments and satisfy the initial/final conditions, we impose constraints on the position, velocity, acceleration, jerk, and snap at the waypoints and intermediate points. This leads to a system of linear equations that can be expressed in matrix form as:

$Mc = b$

where:

- *M* is the constraint matrix, containing coefficients derived from the polynomial derivatives and time values.

- *c* is the coefficient vector, containing the unknown $a_i$ coefficients for all segments.

- *b* is the constraint vector, representing the desired values of position, velocity, acceleration, jerk, and snap at the waypoints and continuity points.


The structure of the M matrix is carefully designed to incorporate the specific constraints of the problem. The first few rows enforce zero velocity, acceleration, and jerk at the start and end points. Subsequent rows enforce position constraints at each waypoint, followed by rows ensuring continuity of derivatives up to the sixth order at the intermediate points between waypoints.

By solving this system of equations, we obtain the polynomial coefficients that define the minimum snap trajectory.


## Pre-requisites

To run the code, you will need the following Python libraries:
- NumPy
- Matplotlib
- mpl_toolkits.mplot3d
These libraries can be installed using pip:

```bash
pip install numpy matplotlib
```


## Usage

**Define Waypoints:** Modify the waypoints list in the main section of the code to specify your desired waypoints in 3D space.
**Set Average Speed:** Adjust the DESIRED_AVERAGE_SPEED variable to control the average speed of the trajectory.
**Run the Code:** Execute the Python script. The code will generate the trajectory, plot it in 3D, and display plots of velocity, acceleration, jerk, and snap over time.
**Analyze Results:** Examine the generated plots and trajectory to understand the characteristics of the minimum snap trajectory.
**Save Trajectory (Optional):** The script provides functionality to save the trajectory coefficients to CSV files for potential use in other applications.

### Further Exploration:
- Experiment with different waypoint configurations and average speeds to observe the effects on the trajectory.
- Modify the code to explore alternative polynomial interpolation methods or constraint settings.
- Integrate the trajectory planning code into a larger robotic or autonomous system simulation.
