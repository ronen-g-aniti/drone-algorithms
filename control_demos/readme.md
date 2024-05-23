# PID Controller Demonstrations

## Table of Contents
1. [Monorotor Controller](#monorotor_controller)
2. [Birotor Controller](#birotor_controller)


## Monorotor Controller <a class="anchor" id="monorotor_controller"></a>

### The Main Idea Behind the Monorotor Controller
The monorotor controller is a simple controller that controls the vertical
position of a monorotor. The monorotor controller is implemented in the
`monorotor_controller.py` file. The controller is implemented using the
following equations:

- PD controller with feedforward:

$$
u = k_p(z_{target} - z_{estimated})+k_d(\dot{z}_{target} - \dot{z}_{estimated}) + u_{ff}
$$

- PID controller with feedforward

$$
u = k_p(z_{target} - z_{estimated})+k_d(\dot{z}_{target} - \dot{z}_{estimated}) + k_i \int_0^{t}(z_{target} - z_{estimated})d\tau + u_{ff}
$$




### Dependencies
This part of the project depends on Numpy and Matplotlib. If you don't have them
installed, you can install them using the following commands:

```bash
pip install numpy
pip install matplotlib
```

### Running the Monorotor Controller
To view the monorotor controller demonstration, navigate to the current
directory in your terminal and run the following command:

```bash
python monorotor_controller.py
```


## Birotor Controller <a class="anchor" id="birotor_controller"></a>

This part of the project depends on Numpy and Matplotlib. If you don't have them
installed, you can install them using the following commands:

```bash
pip install numpy
pip install matplotlib
```

To view the birotor controller demonstration, navigate to the current
directory in your terminal and run the following command:

```bash
python birotor_controller.py
```