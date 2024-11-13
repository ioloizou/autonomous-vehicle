# Lab 2: Using OpenStreetMap for Autonomous Vehicle Local Planners

## Objectives:
1. Understand the fundamentals of OpenStreetMap (OSM) maps.
2. Develop an understanding of local planners for autonomous vehicles.
3. Gain practical experience in designing, implementing, and testing local planners.

## Description:
This lab provides an understanding and experience with OpenStreetMap maps and the development of local planners for autonomous vehicles. You will learn how to load and visualize OSM maps and implement pathfinding algorithms in a simulated environment.

## Steps:
1. Load and Visualize OSM Maps:
   - Use the osmnx library to load an OSM map for a specified location.
   - Visualize the OSM graph, nodes, and edges for this location.
   - Select an area of interest on https://www.openstreetmap.org to work with.

2. Implement Global Path Finding:
   - Determine a path from an arbitrary origin to a destination using the nx library.
   - Plot this path on the OSM graph.
   - Analyze the chosen path for its feasibility and efficiency.

3. Implement a simplified Dynamic Window Approach (DWA) for local planning 
   - Implement cost functions 
   - Implement evolution model (based on the constant velocity bicycle model)

## Expected Learning Outcomes:
- Comprehensive understanding of OSM maps and their application in autonomous vehicle navigation.
- Ability to design, implement, and test local planners for autonomous vehicles in a simulated environment.
- Skills in analyzing and interpreting simulated navigation data.

## Ressources:
- networkX
- osmnx
- openstreetmap


## Deadline
The deadline for this lab submission is the *TBD* on Hippocampus website 
Upload the following completed files:

- lab2.ipynb
- osmnx_utils.py 
