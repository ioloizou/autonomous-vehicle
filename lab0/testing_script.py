import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import time
from utils import *

# Define functions for point cloud processing

def ground_filtering(pcd):
    normals = np.asarray(pcd.normals)  # Convert to NumPy array
    points = np.asarray(pcd.points)  # Convert to NumPy array
    height = 0.3  # Height threshold

    # Get the norm of x and y of normals    
    norm_x_y = np.linalg.norm(normals[:, :2], axis=1)
    mask = (norm_x_y < 0.4) & (points[:, 2] < height)

    pcd_ground = pcd.select_by_index(np.where(mask)[0])
    return pcd_ground

def objects_filtering(pcd):
    normals = np.asarray(pcd.normals)  # Convert to NumPy array
    points = np.asarray(pcd.points)  # Convert to NumPy array
    height = 0.3  # Height threshold

    # Get the norm of x and y of normals    
    norm_x_y = np.linalg.norm(normals[:, :2], axis=1)
    mask = (norm_x_y > 0.4) | (points[:, 2] > height)

    pcd_objects = pcd.select_by_index(np.where(mask)[0])
    return pcd_objects

def clustering(pcd_objects):
    cluster_ids = np.array(pcd_objects.cluster_dbscan(eps=1.0, min_points=10, print_progress=True))
    cluster_nb = cluster_ids.max()
    colors = plt.get_cmap("tab20")(cluster_ids / (cluster_nb if cluster_nb > 0 else 1))
    colors[cluster_ids < 0] = 0
    pcd_objects.colors = o3d.utility.Vector3dVector(colors[:, :3])

    # Exclude points too close to the origin
    mask = np.linalg.norm(np.asarray(pcd_objects.points), axis=1) > 3
    pcd_objects = pcd_objects.select_by_index(np.where(mask)[0])

    return pcd_objects, cluster_ids

def cluster_boxes(pcd_objects, cluster_ids):
    # Define size constraints for car-sized bounding boxes. No need for human since it will be smaller
    car_size = np.array([10., 10., 10.])   # Example dimensions (length, width, height)
    
    # Bounding boxes
    boxes = []
    
    # Get the number of clusters
    max_label = cluster_ids.max()
    
    # Iterate through each cluster and compute the bounding box
    for i in range(max_label + 1):
        cluster_indices = np.where(cluster_ids == i)[0]
        cluster = pcd_objects.select_by_index(cluster_indices)
        
        # Compute the oriented bounding box (OBB)
        obb = cluster.get_oriented_bounding_box()
        
        # Check if the bounding box is within the size constraints for humans or cars
        # and if the bounding box is placed lower than 1 meter from the ground
        if np.all(obb.extent <= car_size) and obb.get_min_bound()[2] <= 1.0:
            obb.color = (1, 0, 0)  # Red color for OBB
            boxes.append(obb)
        
    return boxes

def create_coordinate_frame(size=1.0, origin=[0, 0, 0]):
    return o3d.geometry.TriangleMesh.create_coordinate_frame(size=size, origin=origin)

# Main script
if __name__ == "__main__":
    # Initialize Open3D Visualizer
    flag_display = True
    if flag_display:
        vis = o3d.visualization.Visualizer()
        vis.create_window()

        # Add the origin coordinate frame to the visualizer with increased size
        origin_frame = create_coordinate_frame(size=2.0, origin=[0, 0, 0])  # Increase size for better visibility
        vis.add_geometry(origin_frame)

    # Process a single LiDAR scan
    n_frame = 0
    actor = 'ego_vehicle'
    points = get_point_cloud(n_frame, actor)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    # Voxel down sampling
    downpcd = pcd.voxel_down_sample(0.3)

    # Estimate normals
    downpcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.4, max_nn=100))
    downpcd.colors = o3d.utility.Vector3dVector(np.abs(np.array(downpcd.normals)))

    # Filter ground points
    pcd_ground = ground_filtering(downpcd)

    # Filter objects points
    pcd_objects = objects_filtering(downpcd)

    # Clustering
    pcd_objects, cluster_ids = clustering(pcd_objects)

    # Get bounding boxes for the clusters
    boxes = cluster_boxes(pcd_objects, cluster_ids)

    # Visualize the point cloud and bounding boxes
    obj_to_display = list(boxes)
    obj_to_display.extend([pcd_objects])

    if flag_display:
        vis.clear_geometries()
        vis.add_geometry(pcd_objects)
        vis.add_geometry(origin_frame)
        for box in boxes:
            vis.add_geometry(box)
        vis.poll_events()
        vis.update_renderer()
        time.sleep(0.1)
