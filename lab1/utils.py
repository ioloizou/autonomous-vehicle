import numpy as np
import open3d as o3d

import os
from os import path as osp

import pickle
import glob
import matplotlib.pyplot as plt

from geometry import make_tf, apply_tf

# ---------------------------------------------
# ---------------------------------------------
# --------DO NOT MODIFY BELOW THIS-------------
# ---------------------------------------------
# ---------------------------------------------

CLASS_NAMES = ['car','truck','motorcycle', 'pedestrian']
CLASS_COLORS = plt.cm.rainbow(np.linspace(0, 1, len(CLASS_NAMES)))[:, :3]
CLASS_NAME_TO_COLOR = dict(zip(CLASS_NAMES, CLASS_COLORS))
CLASS_NAME_TO_INDEX = dict(zip(CLASS_NAMES, range(len(CLASS_NAMES))))

# Path extraction
root_path = "/home/ioloizou/autonomous-vehicles/scenario1"

scenario = "Town01_type001_subtype0001_scenario00003"  


file_list = glob.glob(osp.join(root_path,
                                        'ego_vehicle', 'label', scenario) + '/*')
frame_list = []

with open(osp.join(root_path, "meta" ,scenario+ '.txt'), 'r') as f:
            lines = f.readlines()
line = lines[2]
agents =  [int(agent) for agent in line.split()[2:]]

for file_path in file_list:
    frame_list.append(file_path.split('/')[-1].split('.')[0])
frame_list.sort()
# ---------------------------------------------
# ---------------------------------------------
# --------DO NOT MODIFY ABOVE THIS-------------
# ---------------------------------------------
# ---------------------------------------------

def get_actor_T_world(actor, n_frame):

    frame = frame_list[n_frame]
    with open(osp.join(root_path, actor ,'calib',scenario, frame + '.pkl'), 'rb') as f:
        calib_dict = pickle.load(f)
    actor_tf_world = np.array(calib_dict['ego_to_world'])
    lidar_tf_actor = np.array(calib_dict['lidar_to_ego'])
    
    tf =  lidar_tf_actor @ actor_tf_world 
    trans = tf[:3,3]
    if actor == 'infrastructure':
        trans[2] += 2.0
    rot = tf[:3,:3]

    return make_tf(trans,rot) 

def get_sensor_T_actor(actor, n_frame):
    frame = frame_list[n_frame]
    with open(osp.join(root_path, actor ,'calib',scenario, frame + '.pkl'), 'rb') as f:
        calib_dict = pickle.load(f)
    lidar_tf_actor = np.array(calib_dict['lidar_to_ego'])

    tf =  lidar_tf_actor 
    trans = tf[:3,3]
    rot = tf[:3,:3]

    return make_tf(trans,rot) 

def get_point_cloud(n_frame, actor):

    frame = frame_list[n_frame] 
    lidar_data = np.load(root_path +  '/' + actor + '/lidar01/' + scenario +'/' + frame + '.npz')['data']
    lidar_T_actor = get_sensor_T_actor(actor, n_frame)
    lidar_data_actor = apply_tf(lidar_T_actor, lidar_data) #in actor frame
 
    return lidar_data_actor

def get_available_point_clouds(n_frame, actors):
    '''
    :param n_frame: 
    :param actors:
    :return: (N, 8) - x, y, z, l, w, h, yaw, class

    This function is used to get all point clouds available in ego frame
    '''
    ego_to_world = get_actor_T_world(actors[0], n_frame) # the transformation from ego frame to world frame
    merged_pc = get_point_cloud(n_frame, actors[0]) #in ego frame

    # TODO: retrieve point clouds in actor frame for all actors and merge them into one point cloud in ego frame
    for actor in actors[1:]:
        # TODO: map `lidar_data_actor` from actor frame to ego frame
        lidar_data_actor = get_point_cloud(n_frame, actor)
        actor_to_world = get_actor_T_world(actor, n_frame)
        world_to_ego = np.linalg.inv(ego_to_world)
        lidar_data_ego = apply_tf(world_to_ego @ actor_to_world, lidar_data_actor)
        merged_pc = np.concatenate((merged_pc, lidar_data_ego), axis=0)

    return merged_pc

def get_boxes_in_sensor_frame(n_frame, actor):

    frame = frame_list[n_frame] 
    with open(osp.join(root_path,  actor ,'label',scenario, frame + '.txt'), 'r') as f:
        lines = f.readlines()
    
    boxes = []
    for line in lines[1:]:
        line = line.split()
        if line[-1] == 'False':
            continue
        box = np.array([float(line[1]), float(line[2]), float(line[3]), float(line[4]), float(line[5]), float(line[6]), float(line[7]), CLASS_NAME_TO_INDEX[line[0]]])
        # cx, cy, cz, l, w, h, yaw, class
        boxes.append(box)
    return boxes

def get_boxes_in_actor_frame(n_frame, actor): # TODO
    '''
    :param n_frame: 
    :param actor:
    :return: (N, 8) - x, y, z, l, w, h, yaw, class

    This function is used to get boxes detected by the actor in actor frame
    '''

    boxes = get_boxes_in_sensor_frame(n_frame, actor)
    boxes = np.array(boxes).reshape(-1,8) #in sensor frame

    # print(boxes[:, 3:])

    # TODO: map `boxes` from sensor frame to actor frame
    sensor_to_actor = get_sensor_T_actor(actor, n_frame)
    boxes_xyz = apply_tf(sensor_to_actor, boxes[:, :3])

    # print(boxes_xyz.shape)
    # print(boxes[:, 3:].shape)

    # Concatenate the remaining part of the boxes l, w, h, yaw, class
    boxes = np.concatenate((boxes_xyz, boxes[:, 3:]), axis=1)

    return boxes

def get_available_boxes_in_ego_frame(n_frame, actors):
    '''
    :param n_frame: 
    :param actors: a list of actors, the first one is ego vehicle
    :return: (N, 8) - x, y, z, l, w, h, yaw, class

    This function is used to get all available boxes by the actors in ego frame
    '''

    boxes = get_boxes_in_actor_frame(n_frame, actors[0]) #in ego frame
    boxes = np.array(boxes).reshape(-1,8)
    ego_to_world = get_actor_T_world(actors[0], n_frame)
    available_boxes_in_ego_frame = boxes

    # TODO : retrieve boxes in ego frame for all actors
    for actor in actors[1:]:
        actor_to_world = get_actor_T_world(actor, n_frame)
        world_to_ego = np.linalg.inv(ego_to_world)
        boxes = get_boxes_in_actor_frame(n_frame, actor)
        
        # Transform box positions to ego frame
        boxes[:, :3] = apply_tf(world_to_ego @ actor_to_world, boxes[:, :3])
        
        # Transform yaw angles to ego frame
        actor_to_ego_rot = (world_to_ego @ actor_to_world)[:3, :3]
        boxes[:, 6] += np.arctan2(actor_to_ego_rot[1, 0], actor_to_ego_rot[0, 0])
        
        available_boxes_in_ego_frame = np.concatenate((available_boxes_in_ego_frame, boxes), axis=0)


    # Group the boxes by center position if they are close and then take the mean of the boxes to get the final boxes
    # This is to remove duplicate boxes
    unique_boxes = []
    for box in available_boxes_in_ego_frame:
        if len(unique_boxes) == 0:
            unique_boxes.append(box)
        else:
            for i, unique_box in enumerate(unique_boxes):
                if np.linalg.norm(box[:3] - unique_box[:3]) < 1.0:
                    unique_boxes[i][:6] = (unique_box[:6] + box[:6]) / 2
                    unique_boxes[i][6] = np.arctan2(
                        (np.sin(unique_box[6]) + np.sin(box[6])) / 2,
                        (np.cos(unique_box[6]) + np.cos(box[6])) / 2
                    )
                    unique_boxes[i][7] = unique_box[7]  # Keep the class the same
                    break
            else:
                unique_boxes.append(box)
    
    return np.array(unique_boxes)

def filter_points(points: np.ndarray, range: np.ndarray):
    '''
    points: (N, 3) - x, y, z
    range: (6,) - xmin, ymin, zmin, xmax, ymax, zmax

    return: (M, 3) - x, y, z
    This function is used to filter points within the range
    '''
    # TODO: filter points within the range
    
    xmin, ymin, zmin, xmax, ymax, zmax = range
    mask = (points[:,0] >= xmin) & (points[:,0] <= xmax) & (points[:,1] >= ymin) & (points[:,1] <= ymax) & (points[:,2] >= zmin) & (points[:,2] <= zmax)
    filtered_points = points[mask]

    return filtered_points

