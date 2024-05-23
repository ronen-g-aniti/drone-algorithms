 # Free Space Construction Codes & Demonstration

 ## Table of Contents

1. [Introduction](#Introduction)
2. [Installation](#Installation)
3. [Usage](#Usage)

## Introduction <a class = "anchor" id = "Introduction"></a>
The files in this folder demonstrate the concept of constructing free space for a robot.

Free space is the space in which a robot can move without being in collision with obstacles.

The obstacle geometry data, housed in "colliders.csv", found in the "map_data" subdirectory, is used by the functions and methods
in this project to aid in constructing the representation of free space. 

The first file, "environment_data.py", contains code to read in the obstacle geometry information. 
The file contains a class called "EnvironmentData", which, provided the obstacle data file, creates a structured representation of the obstacle data to be used by other modules of this project.

The second file, "lattice.py", contains code to construct a lattice representation of the free space, provided the structured obstacle data, the EnvironmentData object.
The file contains a class called "CubicLattice", which creates a discretized model of free space using a 3D cubic lattice arrangement.


## Developing Some Mathematical Logic

Both files from this project, "environment_data.py" and "lattice.py", leverage mathematical logic that
I've formulated to aid in constructing and representing free space. The following list of considerations
describe some of the main parts of my approach. 

1. Describe the set of all 3D points in the environment. 

$$
E = \{(x,y,z) \in \mathbb{R}^3 \mid x_{min} \leq x \leq x_{max}, y_{min} \leq y \leq y_{max}, z_{min} \leq z \leq z_{max} \}
$$

2. Consider that obstacles, in this project, are defined as 3D boxes with center positions and halfsizes in each of the 
three coordinate directions.
$$
o_i = [(x_c, y_c, z_c), (h_x, h_y, h_z)]
$$

3. One obstacle subvolume can thus be described as the set of all 3D points that it comprises:
$$
O_i = \{(x,y,z) \in \mathbb{R}^3 \mid x_{c,i} - h_{x,i} \leq x \leq x_{c,i} + h_{x,i} ,  y_{c,i} - h_{y,i}  \leq y \leq y_{c,i} + h_{y,i} , z_{c,i} - h_{z,i}  \leq y \leq z_{c,i} + h_{z,i} \}
$$

4. Each obstacle subvolume can be expanded to include padding for a safety margin around the obstacle:
$$
O_{s,i} = \{(x,y,z) \in \mathbb{R}^3 \mid x_{c,i} - h_{x,i} - s \leq x \leq x_{c,i} + h_{x,i} + s,  y_{c,i} - h_{y,i} - s \leq y \leq y_{c,i} + h_{y,i} + s, z_{c,i} - h_{z,i} - s \leq y \leq z_{c,i} + h_{z,i} + s\}
$$
5. The set of all obstacle subvolumes can be constructed by simply taking the union of all the component obstacle subvolumes:

$$
O_s = O_{s,1} \cup O_{s,2} \cup O_{s,3} \cup ... \cup O_{s,N}
$$
6. Free space can thus be described as the set of all 3D points in the environment that are not bounded by any obstacle subvolume:

$$
F = E - O_s
$$

### The Mathematical Representation of the Lattice

The lattice is mathematically described as a set of discrete points in $\mathbb{R}^3$, constructed within the environment as follows:

$$
L = \{(x_i, y_j, z_k) \mid x_i = x_{\text{min}} + i \cdot \delta x, y_j = y_{\text{min}} + j \cdot \delta y, z_k = z_{\text{min}} + k \cdot \delta z, i, j, k \in \mathbb{N}_0 \}
$$




## Installation <a class = "anchor" id = "Installation"></a>
This project requires the following dependencies:
- Matplotlib
- Numpy
- Scipy
- UTM

To install the dependencies, run the following commands:
```bash
pip install matplotlib numpy scipy utm
```

## Usage <a class = "anchor" id = "Usage"></a>
Each file in this folder can be run independently to demonstrate the concept of constructing a free space model for an aerial robot.
- "environment_data.py" will parse the obstacle data, create a structured EnvironmentData object, and visualize the obstacles in the environment.
- "lattice.py" will create and visualize a lattice representation of free space. 

To run the files, open a terminal window and run the following commands from this folder's directory: 
```bash
python environment_data.py
python lattice.py
```

Experiment with setting different parameters in the code to see how the free space representation changes.

