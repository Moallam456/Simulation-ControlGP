# Kinematics Interface

Version: 0.1  
Reference Robot: UR5  
Purpose: Shared kinematics interface between the Simulation and Control teams

---

## 1. Purpose

This document defines the common kinematic conventions, inputs, outputs, and function interfaces that shall be used by both the Simulation and Control teams.

The Control team is responsible for implementing:

- Forward Kinematics (FK)
- Inverse Kinematics (IK)
- Jacobian

The Simulation team is responsible for:

- Maintaining the reference robot model
- Providing the shared robot configuration
- Providing the shared MATLAB robot build for visualization/debugging
- Independently validating the Control team's FK, IK, and Jacobian implementations
- Performing workspace, reachability, singularity, and manipulability analysis

The current robot used for algorithm development and validation is the UR5 reference robot.

This UR5 model is not the final welding robot. It is used as a known reference configuration so that the kinematic algorithms can be developed and validated before the final welding robot geometry is defined.

---

## 2. Shared Files

The Control team will receive:

```text
+config/
    UR5.m

+robotmodel/
    buildRobot.m

KinematicsInterface.md




