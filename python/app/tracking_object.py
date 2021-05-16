import pyrealsense2 as rs
import numpy as np
import open3d as o3d
import random
import shortuuid
import time



class TrackingObject:
    def __init__(self):
        self.points = o3d.utility.Vector3dVector()
        self.status = "candidate"  #uninit, candidate, active, occluded, lost
        self.id = shortuuid.uuid()[0:4]
        self.bounding_box = o3d.geometry.AxisAlignedBoundingBox()
        self.speed_vector = [0.0,0.0,0.0,0.0]
        self.last_position = [0.0,0.0,0.0]
        self.last_measurement_timestamp = time.time()
        self.volume = 0
        self.colors = []
        self.state_counter = 0
        self.alive_frames = 0
        for i in range(0,3):
            self.colors.append(random.uniform(0.0,1.0))



    def calc2DBoundingBox(self):
        min_y = 0
        max_y = 720
        min_x = 0
        max_x = 1280
        for point in self.points:
            if point.x < min_x:
                min_x = point.x
            if point.x > min_x:
                pass
            if point.y < min_y:
                pass
            if point.y > max_y:
                pass

    def bbox2_3D(self):

        r = np.any(img, axis=(1, 2))
        c = np.any(img, axis=(0, 2))
        z = np.any(img, axis=(0, 1))

        rmin, rmax = np.where(r)[0][[0, -1]]
        cmin, cmax = np.where(c)[0][[0, -1]]
        zmin, zmax = np.where(z)[0][[0, -1]]

        return rmin, rmax, cmin, cmax, zmin, zmax

    def calc_bounding_box(self,intrinsics):
        self.bounding_box = o3d.geometry.AxisAlignedBoundingBox.create_from_points(self.points)
        self.bounding_box.color = self.colors
        self.volume = self.bounding_box.volume()
        # x_coordinates, y_coordinates, z_coords = zip(*self.points)
        # #x_min = rs.rs2_project_point_to_pixel(intrinsics,min(x_coordinates))
        # #y_min = rs.rs2_project_point_to_pixel(intrinsics,min(y_coordinates))
        # #x_max = rs.rs2_project_point_to_pixel(intrinsics,max(x_coordinates))
        # #y_max = rs.rs2_project_point_to_pixel(intrinsics, max(y_coordinates))
        # point_1 = rs.rs2_project_point_to_pixel(intrinsics,[min(x_coordinates),min(y_coordinates),min(z_coords)])
        # point_2 = rs.rs2_project_point_to_pixel(intrinsics,[max(x_coordinates),max(y_coordinates),max(z_coords)])
        #
        # self.bounding_box = [(abs(int(point_1[0])),abs(int(point_1[1]))),(abs(int(point_2[0])),abs(int(point_2[1])))]

    def update(self, obj):
        self.points = obj.points
        self.bounding_box = obj.bounding_box
        self.volume = obj.volume
        self.bounding_box.colors = self.colors

    def updateState(self,new_state):
        self.status = new_state
        self.state_counter = 0





