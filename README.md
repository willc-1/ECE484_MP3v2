# MP3 Environment and Localization Enhancements

This document details the modifications made to the MP3 environment for the autonomous vehicle project (SP25 version).

## Environment Enhancements

To improve the convergence speed and accuracy of the particle filter localization algorithm, we modified the simulation environment to be less symmetrical and provide more unique features for localization:

- **New obstacles introduced**: rotated walls and boxes, L-shaped, Z-shaped, and other non-symmetrical structures, distinctive features near the environment boundaries,cross-shaped structure in the center of the environment as a reference point
These changes help the particle filter converge faster by providing unique sensor readings at different positions in the environment.

## SDF Reader Improvements

The original SDF reader could only process axis-aligned obstacles correctly. We implemented a hybrid approach that handles both axis-aligned and rotated obstacles:

- **Rotation handling**: Added support for processing obstacles with arbitrary rotation angles.
- **Polygon-based representation**: Implemented algorithms to convert rotated rectangles into grid cell representations.
- **Hybrid processing approach**: Used the original method for axis-aligned obstacles and the new polygon method for rotated obstacles.
- **Coordinate system alignment**: Improved the transformation between Gazebo and internal coordinate systems.

## Position Setting Modifications

To ensure collision-free navigation in the enhanced environment:

- **Updated waypoint system**: Modified the `vehicle.py` waypoint list to avoid collisions with the new obstacles.
- **Path recording tool**: Created a tool to record vehicle positions while manually driving through the environment.
- **Environment verification**: Implemented visualization tools to verify the correct processing of the updated environment.

## Another locolization tryout: Vertical edge detection using camera
- Implemented camera_processing script for extracting vertical edge information in the environment
- Added testing scripts for edge detection functionality
- Note: Not working ideally since lack of feature information in gazebo ECEB environment. Need to update more features to extract. Forks for implementing this functionality are welcome!
