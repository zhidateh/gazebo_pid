# **Simulation of PID controller on Gazebo ROS using Husky robot**

### Objective
This project is to use PID controller to control the motion of robot in a virtual environment to understand how PID works. The husky robot can be controlled through two input commands - forward/backward motion and steering motion. The simulation world consists of 1 pillar which is our target destination.

The scripts are written in C++.

### Parameters of PID controller

**Description of PID values in PID control**

* **P** (proportional) accounts for present values of the error. For example, if the error is large and positive, 
the control output will also be large and positive.

* **I** (integral) accounts for all past values of the error. For example, if the current output is not sufficiently 
strong, the integral of the error will accumulate over time, and the controller will respond by 
applying a stronger action.

* **D** (differential) accounts for possible future trends of the error, based on its current rate of change.

**Parameters**

* PID parameters used for **forward/backward motion** is stored in config.yaml: 

    * Kp_f 
    * Ki_f 
    * Kd_f

* PID parameters used for **steering motion** is stored in config.yaml:

    * Kp_a 
    * Ki_a 
    * Kd_a








[//]: # (Image References)
[image1]: ./data/1.png
[gif1]: ./data/steady.gif
[gif2]: ./data/speedy.gif


