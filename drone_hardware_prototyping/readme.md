# Monorotor Control

I implement my monorotor PID controller on an Arduino development board. In this experiment, I add PID control codes to an Arduino sketch. I also add a class to model the dynamics of a monorotor drone. I have code that gets the trajectory from my ground station PC before flight. 

## Physical Components
- DC motor
- Transistor
- Diode
- Resistor
- Arduino Uno
- Breadboard
- 9V battery

## Procedure
The ground station PC will house the dynamics class and the trajectory object. The arduino will house the PID control codes, adapted to send a PWM signal to the DC motor. When the sketch runs, the arduino will get the trajectory from the ground station PC and save it in memory as a struct. It will use millis to track time. Based on the current time and the trajectory struct, it will calculate the desired z position and z velocity. It will then use the PID controller to calculate the desired PWM signal to send to the DC motor. 