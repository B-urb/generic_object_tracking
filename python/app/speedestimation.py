import pyrealsense2 as rs
import numpy as np
import cv2
import open3d as o3d
import copy
import scipy
from app.clustering import clusterDBscanO3D, clusterDBScanScipy, clusterMeanShift, clusterAgglomerative
import matplotlib.pyplot as plt
import time
from app.tracking import processObjects
from app.tracking_object import TrackingObject
from app.assignment import calc_cost_matrix, create_assignment



class SpeedEstimation:
    def __init__(self, intrinsics, config):
        self.objects = []
        self.old_time = time.time()
        self.cycle_count = 0
        self.intrinsics = intrinsics
        self.config = config



    def step(self, points):
        cycle_time_start = time.time()

        # Floor plane removal
        pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
        #pcd.compute_mahalanobis_distance()
        start = time.time()
        plane_model, inliers = pcd.segment_plane(distance_threshold=0.04,
                                                 ransac_n=3,
                                                 num_iterations=200)

        start = time.time()
        labels = clusterDBscanO3D(pcd)
        #labels = clusterDBScanScipy(pcd.points)
        #labels = clusterMeanShift(np.asarray(pcd.points))
        #labels = clusterAgglomerative(np.asarray(pcd.points))

        end = time.time()
        print("Clustering took: " + str((end - start) * 1000) + "ms")
        max_label = labels.max()
        detections = []
        # deal with temporary loss, maybe keepm state(track of recent lost objects), add last seen counter
        for i in range(0, max_label + 1):
            detections.append(TrackingObject())

        [a, b, c, d] = plane_model

        pcd = pcd.select_by_index(inliers, invert=True)
        end = time.time()
        # print("Plane segmentation took: " + str((end - start) * 1000) + "ms")

        point_array = points
        for i, point in enumerate(point_array):
            cluster_id = labels[i]
            # print(cluster_id)
            if cluster_id != -1:
                detections[cluster_id].points.append(point)

        # print(f"point cloud has {max_label + 1} clusters")
        for obj in detections:
            if len(obj.points) < 60:  # or len(obj.points) > 2000:
                detections.remove(obj)

        for obj in detections:
            obj.calc_bounding_box(self.intrinsics)

        if len(self.objects) == 0:
            self.objects = detections


        print(f"There are {len(self.objects)} objects and {len(detections)} detections")
        start = time.time()
        cost_matrix = calc_cost_matrix(self.objects, detections)
        assignment_matrix = create_assignment(self.objects, cost_matrix, self.config.max_dist)
        current_time = time.time()

        # TODO: Use kalman predict in case of occlusion, needs detected occlusion
        if len(assignment_matrix) > 0:
            for i, detection_index in enumerate(assignment_matrix):
                if assignment_matrix[i] != -1:
                    if self.objects[i].status == "candidate":
                        self.objects[i].state_counter += 1
                    if self.objects[i].status == "active":
                        self.objects[i].state_counter = 0
                    if self.objects[i].status == "lost":
                        self.objects[i].status = "active"

                    self.objects[i].points = detections[assignment_matrix[i]].points
                    self.objects[i].calc_bounding_box(self.intrinsics)
                    if self.cycle_count % 30 == 0:
                        p1 = self.objects[i].last_position
                        p2 = self.objects[i].bounding_box.get_center()
                        self.objects[i].last_position = p2
                        p2 = detections[assignment_matrix[i]].bounding_box.get_center()


                        distance = scipy.spatial.distance.euclidean(p1, p2)
                        time_elapsed = (current_time - self.objects[i].last_measurement_timestamp)
                        velocity = self.calc_velocity(p1, p2, distance, time_elapsed)
                        self.objects[i].speed_vector = velocity
                        self.objects[i].last_measurement_timestamp = current_time
                        # old_time = current_time

                        # detections.remove(detections[j])
                else:
                    self.objects[i].state_counter += 1

        end = time.time()
        print("Assignment took: " + str((end - start) * 1000) + "ms")

        for i in range(len(detections)):
            if i not in assignment_matrix:
                candidate = detections[i]
                self.objects.append(candidate)



        ########
        ########
        ## Rendering
        ########
        ########
        colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
        colors[labels < 0] = 0
        pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])

        #bb = [x.bounding_box for x in self.objects]
        o3d.visualization.draw_geometries([pcd, *[x.bounding_box for x in self.objects]])
        cv2.waitKey(1)


        # print("Rendering took: " + str((end - start) * 1000) + "ms")
        # objects processing
        self.objects = processObjects(self.objects)

        ####
        ## Write Frame to video
        ####

        cycle_time_end = time.time()
        print("Cycle time: " + str((cycle_time_end - cycle_time_start) * 1000) + "ms")
        self.cycle_count += 1

    def calc_velocity(self, p1, p2, distance, time_elapsed):  # time_elapsed in ms
        vx = 0.0
        vy = 0.0
        vz = 0.0
        speed = 0.0
        if time_elapsed > 0 and distance > 0:
            dx = p2[0] - p1[0]
            dy = p2[1] - p1[1]
            dz = p2[2] - p1[2]
            d = np.sqrt(dx * dx + dy * dy + dz * dz)
            speed = (distance / time_elapsed) * 1000
            vx = dx / d * speed
            vy = dy / d * speed
            vz = dz / d * speed

        return [vx, vy, vz, speed]

    # TODO :'REMOVE HUNGARIAN ASSIGNMENT AND JUST MATCH MIN DISTANCE'
    # TODO: Bounding box overlap and distance criteria
    # TODO: Refactor code and and options for easy visual
    # TODO: Add Kalman Filter and occlusion detection
    # TODO: Change state counter to precentage of frames observed
    # TODO: After refactor, eval different clustering algorithms
    # TODO: get index of point and check different camera distances
    # TODO: get correct units
    # TODO: accumulate average distance over time
    #
    def truncate(self, f, n):
        '''Truncates/pads a float f to n decimal places without rounding'''
        s = '{}'.format(f)
        if 'e' in s or 'E' in s:
            return '{0:.{1}f}'.format(f, n)
        i, p, d = s.partition('.')
        return '.'.join([i, (d + '0' * n)[:n]])

    def calc_point_distance(self, p1, p2):
        squared_dist = np.sum((p1 - p2) ** 2, axis=0)
        dist = np.sqrt(squared_dist)
        return dist

