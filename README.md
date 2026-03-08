# Inverse Kinematics Sketch

> An interactive 6-DOF robotic arm inverse kinematics solver in Processing using Denavit-Hartenberg parameters.

---

## Screenshots / Examples

![processing sketch preview](https://github.com/s4lt3d/IK_Sketch/blob/master/robotic_arm.gif?raw=true)

**Real-world application:**

[![Robotic Arm and Processing](https://img.youtube.com/vi/T0fo-2nNwrg/0.jpg)](https://www.youtube.com/watch?v=T0fo-2nNwrg)

---

## Overview

This Processing sketch demonstrates inverse kinematics for a 6-degree-of-freedom (DOF) robotic arm. Using Denavit-Hartenberg (DH) parameters and matrix operations, it calculates joint angles needed to position the end-effector at a desired location.

---

## Features

- **6-DOF robotic arm** — Full articulated arm with configurable link lengths
- **DH parameters** — Industry-standard representation of arm geometry
- **Interactive control** — Mouse-based end-effector positioning
- **Real-time visualization** — 3D rendering of arm configuration

---

## Usage

1. Install [Processing](https://processing.org/)
2. Copy the code into the Processing IDE
3. Run the program
4. Interact with the mouse:
   - **Move mouse** — Change target position
   - **Click and drag** — Change target end-effector orientation

---

## Mathematics

The solution uses the Denavit-Hartenberg convention to represent joint parameters and forward/inverse transformations.

### Denavit-Hartenberg Parameters

The DH convention defines each joint by four parameters:
- **θ (theta)** — Rotation angle around Z-axis
- **d** — Distance along Z-axis
- **a** — Distance along X-axis (link length)
- **α (alpha)** — Rotation angle around X-axis

These parameters create transformation matrices that chain together to compute end-effector position from joint angles.

### Inverse Kinematics Approach

The sketch calculates joint angles needed to reach a target position by:
1. Using the target end-effector position and orientation
2. Working backwards through the kinematic chain
3. Solving for individual joint angles

---

## Configuration

Link lengths and DH parameters are configurable within the sketch. Modify the arm geometry by adjusting:
- Segment lengths (link1, link2, link3, etc.)
- Joint angle constraints
- DH parameter values

---

## Technical Details

- **Language:** Processing (Java-based)
- **3D Libraries:** Processing's built-in 3D rendering
- **Input:** Mouse-driven target positioning
- **Output:** Real-time 3D visualization of arm configuration
