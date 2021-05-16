import pyrealsense2 as rs
import numpy as np
import cv2
import open3d as o3d
from datetime import datetime
import matplotlib.pyplot as plt
import time
from sklearn.cluster import KMeans



pcd = o3d.io.read_point_cloud("./data/1_cloud.pcd")
#pcd_tree = o3d.geometry.KDTreeFlann(pcd)

# pcd.paint_uniform_color([0.5, 0.5, 0.5])
# pcd.colors[1500] = [1, 0, 0]
# [k, idx, _] = pcd_tree.search_knn_vector_3d(pcd.points[1500], 200)
# np.asarray(pcd.colors)[idx[1:], :] = [0, 0, 1]
#
# [k, idx, _] = pcd_tree.search_radius_vector_3d(pcd.points[1500], 0.2)
# np.asarray(pcd.colors)[idx[1:], :] = [0, 1, 0]
#


# voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd,
#                                                              voxel_size=0.02)
octree = o3d.geometry.Octree.convert_from_point_cloud(pcd, size_expand=0.01)

o3d.visualization.draw_geometries([octree])
#print(pcd.colors[2,2,2])

#o3d.visualization.draw_geometries([downpcd])
