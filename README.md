# Vicharaka Rover - Main Codebase

This repository contains the main codebase for the Vicharaka Rover project, structured to support configuration, description, and simulation in ROS and Gazebo.

## Repository Structure

- **config/**  
  Contains `.yaml` files for passing various parameters to nodes and components.
  
- **description/**  
  Includes all `.xacro` files for URDF modeling, along with plugin descriptions used for the rover's physical and simulation components.
  
- **launch/**  
  Houses all the executable launch files required to run various nodes and functionalities within this package.
  
- **worlds/**  
  Consists of custom Gazebo simulation environments for testing and developing the rover in a virtual world.
  
- **CMakeLists.txt**  
  The CMake configuration for building the package.

- **package.xml**  
  ROS package metadata, including dependencies and version information.

## Dependencies

To install necessary ROS packages, run the following commands:

```bash
sudo apt install ros-<distro>-robot-state-publisher
sudo apt install ros-<distro>-xacro
