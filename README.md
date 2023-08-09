# Inverse Kinematics Demo

## Description:
This code demonstrates a basic inverse kinematics solution for a 6 degrees of freedom (DOF) robotic arm in Processing. Using Denavit-Hartenberg (DH) parameters and matrix operations, it calculates the angles of the arm's joints based on a desired end-effector position. The arm's structure and its parameters, like link lengths and tool lengths, are predefined. The demonstration allows users to interact with the 3D visualization by manipulating the target end-effector position with the mouse.

### Mathematical Basis:
The solution uses the Denavit-Hartenberg (DH) convention to represent the joint parameters and transformations. 

### How to Run:
1. Ensure you have the [Processing environment](https://processing.org/) installed.
2. Copy the provided code into the Processing IDE.
3. Run the program. A 3D visualization of the robot arm will appear.
4. Use the mouse to interact:
   - Click and drag to change the orientation of the target end-effector.
   - Move the mouse without pressing to change the target position.
   
![processing sketch preview](https://github.com/s4lt3d/IK_Sketch/blob/master/robotic_arm.gif?raw=true)

Here's a link to a video which used this sketch to control a robot. 

[![Robotic Arm and Processing](https://img.youtube.com/vi/T0fo-2nNwrg/0.jpg)](https://www.youtube.com/watch?v=T0fo-2nNwrg)
