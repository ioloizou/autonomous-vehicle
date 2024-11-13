
# Lab 1: Collaborative Perception for Autonomous Vehicles

## Objectives:
1. Learn to process and visualize LiDAR data from IRSU for vehicle perception.
2. Understand how to enhance vehicle perception using data from other vehicles and IRSU.
3. Develop skills in constructing an occupancy grid for environmental mapping.

## Description:
This lab focuses on the utilization of Intelligent Road-Side Unit (IRSU) data, particularly in the context of enhanced vehicle perception and environment mapping. Students will work with LiDAR data and learn how to effectively visualize and interpret this data for autonomous vehicle applications.

## Steps:
1. Task 1 - Visualizing LiDAR Data from IRSU:
   - Process and visualize point cloud data from a 32-channel LiDAR.
   - Modify functions in `visualization.py` and `utils.py` to represent the data effectively.

At the end of the first task you should see something like this
![image](https://github.com/CBeaune/ECN_AUVE_labs_students/assets/57994352/82ca7e5f-98a6-4be9-bade-671dff4bb073)


2. Task 2 - Enhancing Vehicle Perception with IRSU Data:
   - Learn how IRSU data can enhance the vehicle's field of view and perception range.
   - Modify and utilize functions in `utils.py` to integrate data from various sources.

At the end of the 2nd task you shouls see something like this
![image](https://github.com/CBeaune/ECN_AUVE_labs_students/assets/57994352/6c16cd43-f48b-45f3-9cd9-f838232d3efb)

3. Task 3 - Building an Occupancy Grid:
   - Construct a Bird's Eye View (BEV) occupancy grid of the environment.
   - Understand the significance of occupancy grids in autonomous vehicle navigation.

At the end of the 3rd task you sould get the following results
![image](https://github.com/CBeaune/ECN_AUVE_labs_students/assets/57994352/e6c18298-6f27-40da-8414-c6169de5d8b0)


![image](https://github.com/CBeaune/ECN_AUVE_labs_students/assets/57994352/c3dfa555-c7e6-41ec-86ac-7e926bf70d4b)



## Expected Learning Outcomes:
- Visualizing and interpreting LiDAR data.
- Knowledge of how IRSU data can augment single vehicle perception.
- Occupancy grid construction.

## Ressources:
- networkX
- osmnx
- openstreetmap

## Deadline
The deadline for this lab submission is the *TBD* on Hippocampus website 
Upload the following completed files:

- lab1.ipynb
- utils.py 
- geometry.py
- visualization.py
