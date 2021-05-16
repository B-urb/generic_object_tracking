import pyrealsense2 as rs
import numpy as np
import cv2
import math
import open3d as o3d
import copy
import scipy
#import matplotlib.pyplot as plt
import time
from app.speedestimation import SpeedEstimation
from app.samplegrid import SampleGrid
from app.visualizer import Visualizer
from app.vector import Vector
import pyrealsense2 as rs
from mpl_toolkits.mplot3d import Axes3D
import matplotlib
matplotlib.use('TkAgg')
plt = matplotlib.pyplot

NUMBER_OF_OBJECTS = 100
MAX_DIST = 0.5
CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720

high_res_config = {
"IMAGE_WIDTH":1280,
          "IMAGE_HEIGHT":720,
          "samplesX":32,
          "samplesY":32,
            "maxDist":0.5,
            "NUMBER_OF_OBJECTS":100,
          "pruneThreshold":0.7,
          "greedThreshold":0.01,
          "mergeThreshold":0.1,
    "floodThreshold":0.01
}

low_res_config = {
"IMAGE_WIDTH":640,
          "IMAGE_HEIGHT":480,
          "samplesX":32,
          "samplesY":32,
            "maxDist":0.5,
            "NUMBER_OF_OBJECTS":100,
          "pruneThreshold":0.8,
          "greedThreshold":0.01,
          "mergeThreshold":0.1,
    "floodThreshold":0.01
}


def main():

    app_config = low_res_config
    # objects = np.empty(NUMBER_OF_OBJECTS, dtype=object)
    objects = []
    old_time = time.time()
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter('./data/output.avi', fourcc, 30.0, (CAMERA_WIDTH, CAMERA_HEIGHT))
    pipeline = rs.pipeline()
    queue = rs.frame_queue(5000, keep_frames=True)
    config = rs.config()

    #Visualizer(CAMERA_WIDTH,CAMERA_HEIGHT, "Visualizer", resizable=True)

    samplegrid = SampleGrid(app_config)
    samplegrid.initialize()



    # note: using 640 x 480 depth resolution produces smooth depth boundaries
    #       using rs.format.bgr8 for color image format for OpenCV based image visualization
    config.enable_stream(rs.stream.depth, app_config["IMAGE_WIDTH"], app_config["IMAGE_HEIGHT"], rs.format.z16, 30)
    config.enable_stream(rs.stream.color, app_config["IMAGE_WIDTH"], app_config["IMAGE_HEIGHT"], rs.format.rgb8, 30)

    config.enable_record_to_file("./data/record.bag")
    #config.enable_device_from_file("./data/realsense.bag", repeat_playback=True)
    #config.enable_device_from_file("./data/record_homeroom_low.bag", repeat_playback=True)
    # Start streaming
    profile = pipeline.start(config, queue)
    depth_sensor = profile.get_device().first_depth_sensor()
    #depth_sensor.set_option(rs.option.max_distance, 4)

    # Getting the depth sensor's depth scale (see rs-align example for explanation)
    depth_scale = depth_sensor.get_depth_scale()

    # We will not display the background of objects more than
    #  clipping_distance_in_meters meters away
    clipping_distance_in_meters = 4  # 3 meter
    clipping_distance = clipping_distance_in_meters / depth_scale

    # rs400::advanced_mode
    # advanced_device(camera.getDevice());
    # auto
    # depth_table = advanced_device.get_depth_table();
    # depth_table.depthClampMax = 1300; // 1
    # m30 if depth
    # unit
    # at
    # 0.001
    # advanced_device.set_depth_table(depth_table);



    # filters
    # decimation filter
    decimation = rs.decimation_filter()
    decimation.set_option(rs.option.filter_magnitude, 4)

    # spatial filter
    spatial = rs.spatial_filter()
    spatial.set_option(rs.option.filter_magnitude, 5)
    spatial.set_option(rs.option.filter_smooth_alpha, 1)
    spatial.set_option(rs.option.filter_smooth_delta, 50)

    temporal = rs.temporal_filter()

    hole_filling = rs.hole_filling_filter()

    depth_to_disparity = rs.disparity_transform(True)
    disparity_to_depth = rs.disparity_transform(False)

    # Create an align object
    # rs.align allows us to perform alignment of depth frames to others frames
    # The "align_to" is the stream type to which we plan to align depth frames.
    align_to = rs.stream.color
    align = rs.align(align_to)

    transformation_matrix_1 = [[0,0, -1], [0,1, 0], [1, 0,0]]#, [0, 0, 0, 1]]
    transformation_matrix_2 = [[1,0, 0], [0,0, -1], [0, 1,0]]#, [0, 0, 0, 1]]

    #transformation_matrix = [[-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]]

    #inv_transform = np.linalg.inv(transformation_matrix)

    # Streaming loop
    frame_count = 0
    intrinsics = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()

    speedestimation = SpeedEstimation(intrinsics, config)
    while frame_count < 1000:

        cycle_time_start = time.time()

        # Get frameset of color and depth
        # frames = pipeline.wait_for_frames()
        start = time.time()
        frames = queue.wait_for_frame().as_frameset()
        current_time = frames.get_frame_metadata(rs.frame_metadata_value.time_of_arrival)


        # Align the depth frame to color frame
        aligned_frames = align.process(frames)

        # Get aligned frames
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        if frame_count < 60:
            #print(frame_count)
            frame_count += 1
            continue

        #depth_frame = decimation.process(depth_frame)
        # depth_frame = depth_to_disparity.process(depth_frame)
        # depth_frame = spatial.process(depth_frame)
        # depth_frame = temporal.process(depth_frame)
        # depth_frame = disparity_to_depth.process(depth_frame)
        # depth_frame = hole_filling.process(depth_frame)

        # Validate that both frames are valid
        if not depth_frame or not color_frame:
            continue

        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        end = time.time()
        print("Frame Post Processing took: " + str((end - start) * 1000) + "ms")

        #############
        ####
        ## Create Point Cloud
        ####
        #############

        start = time.time()
        pc = rs.pointcloud()
        points = pc.calculate(depth_frame)
        point_array = np.asarray(points.get_vertices(2))


        end = time.time()
        print("Pointcloud generation took: " + str((end - start) * 1000) + "ms")
        print(point_array.size)

        # Transformation code
        # newrow = np.ones((point_array.shape[0],1))
        # point_array = np.append(point_array,newrow,axis=1)
        point_array = np.dot(point_array,transformation_matrix_1)
        point_array = np.dot(point_array,transformation_matrix_2)

        # point_array = np.delete(point_array, np.s_[3:4], axis=1)

        #pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(point_array))
        #res = pcd.estimate_normals()
        #print(res)
        #o3d.visualization.draw_geometries([pcd])

        ## FLood fill
        start = time.time()
        #samplegrid.update(np.asarray(pcd.points))
        samplegrid.update(point_array)
        end = time.time()
        print("update took: " + str((end - start) * 1000) + "ms")

        start = time.time()
        floorplane = samplegrid.findFloor()
        end = time.time()
        print("Floor detection: " + str((end - start) * 1000) + "ms")


        # normal = np.array(floorplane.n)
        # point = np.array(floorplane.p)
        # d = -point.dot(normal)
        # xx, yy = np.meshgrid(range(10), range(10))
        #
        # # calculate corresponding z
        # z = (-normal[0] * xx - normal[1] * yy - d) * 1. / normal[2]
        #
        # # plot the surface
        # plt3d = plt.figure().gca(projection='3d')
        # plt3d.plot_surface(xx, yy, z)
        # plt.show()
        # #plt.savefig("planeimg.png")

        trans = transformFromGroundPlane(floorplane.n,floorplane.p)
        indices = []
        newrow = np.ones((point_array.shape[0], 1))
        point_array = np.append(point_array, newrow, axis=1)
        point_array = np.dot(point_array, np.array(trans))
        point_array = np.delete(point_array, np.s_[3:4], axis=1)

        trans = [j for sub in trans for j in sub]
        # for i, arr in enumerate(point_array):
        #     arr[0] = trans[0] * arr[0] + trans[4] * arr[1]+ trans[8] * arr[2] + trans[12]
        #     arr[1] = trans[1] * arr[0] + trans[5] * arr[1] + trans[9] * arr[2] + trans[13]
        #     arr[2] = trans[2] * arr[0] + trans[6] * arr[1] + trans[10] * arr[2] + trans[14]
        #point_array = np.delete(point_array, indices, axis=0)
        end = time.time()
        # print("Removal of clipping: " + str((end - start) * 1000) + "ms")
        # print(point_array.size)



        start = time.time()
        #speedestimation.step(point_array)
        #speedestimation.step(pcd.points)

        end = time.time()
        print("Speed estimation took: " + str((end - start) * 1000) + "ms")

        # for obj in objects:
        #     if obj.status != "active":
        #         continue
        #
        #     corners = np.asarray(obj.bounding_box.get_box_points())
        #     extent = obj.bounding_box.get_extent()
        #     pixels = []
        #     for point in corners:
        #         pixel = rs.rs2_project_point_to_pixel(intrinsics, point.tolist())
        #         pixel = [int(coord) for coord in pixel]
        #         pixels.append(pixel)
        #
        #     cv2.rectangle(color_image, tuple(pixels[1]), tuple(pixels[2]), (0, 255, 0), 2)
        #
        #
        #     label = "Obj-ID: "+str(obj.id) +"\n" \
        #             + "vx: " + str(round(obj.speed_vector[0],2))  + " m/s \n"  \
        #             + "vy: " + str(round(obj.speed_vector[1], 2)) + " m/s \n" \
        #             + "vz: " + str(round(obj.speed_vector[2], 2))+ " m/s \n" \
        #             + "speed  " + str(round(obj.speed_vector[3],2)) + "m/s \n"
        #     position = tuple(pixels[5])
        #     y0,dy = position[1], 12
        #     for i, line in enumerate(label.split('\n')):
        #         y = y0 + i * dy
        #         #cv2.putText(img, line, (50, y), cv2.FONT_HERSHEY_SIMPLEX, 1, 2)
        #         cv2.putText(color_image, line,(position[0]+10,y) , cv2.FONT_HERSHEY_COMPLEX, 0.5, (255, 255, 0), 1)
        #
        #
        # end = time.time()
        # print("2D projection time: " + str((end - start) * 1000) + "ms")
        #
        # cv2.putText(color_image, str(frame_count), (50, 50), cv2.FONT_HERSHEY_COMPLEX, 2, (255, 255, 0), 1)
        # out_img = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)
        # cv2.imshow('img', out_img)
        # k = cv2.waitKey(1)
        # mesh = o3d.geometry.TriangleMesh.create_coordinate_frame()
        # pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(point_array))
        # pcd.transform(trans)
        # indices = []
        # for i, arr in enumerate(np.asarray(pcd.points)):
        #     if arr[2] < 0.05:
        #         indices.append(i)

        # point_array = np.delete(pcd.points, indices, axis=0)
        # pcd.points = o3d.utility.Vector3dVector(point_array)
        # o3d.visualization.draw_geometries([pcd, mesh])
        cycle_time_end = time.time()
        print("Cycle time: " + str((cycle_time_end - cycle_time_start) * 1000) + "ms")
    out.release()
    pipeline.stop()


def rotation_matrix(axis, theta):
    """
    Return the rotation matrix associated with counterclockwise rotation about
    the given axis by theta radians.
    """
    axis = np.asarray(axis)
    axis = axis / math.sqrt(np.dot(axis, axis))
    a = math.cos(theta / 2.0)
    b, c, d = -axis * math.sin(theta / 2.0)
    aa, bb, cc, dd = a * a, b * b, c * c, d * d
    bc, ad, ac, ab, bd, cd = b * c, a * d, a * c, a * b, b * d, c * d
    return np.array([[aa + bb - cc - dd, 2 * (bc + ad), 2 * (bd - ac)],
                     [2 * (bc - ad), aa + cc - bb - dd, 2 * (cd + ab)],
                     [2 * (bd + ac), 2 * (cd - ab), aa + dd - bb - cc]])


def transformFromGroundPlane(n, p):
    matrix = []#np.zeros((4,4))
    for i in range(0,4):
        matrix.append([0,0,0,0])
    up = Vector(0,0,1)
    axis = n^up
    axis = axis.normalize()
    angle = n.angleTo(up)
    z = n*(-p)

    c = math.cos(angle)
    s = math.sin(angle)
    t = 1.0 -c

    matrix[0][0] = c + axis[0]*axis[0]*t
    matrix[1][1] = c + axis[1] * axis[1] * t
    matrix[2][2] = c + axis[2] * axis[2] * t

    tmp1 = axis[0]*axis[1]*t
    tmp2 = axis[2]*s
    matrix[0][1] = tmp1 + tmp2
    matrix[1][0] = tmp1 - tmp2
    tmp1 = axis[0]*axis[2]*t
    tmp2 = axis[1] *s
    matrix[1][2] = tmp1 - tmp2
    matrix[2][1] = tmp1 + tmp2
    matrix[3][0] = 0.0
    matrix[3][1] = 0.0
    matrix[3][2] = z
    matrix[0][3] = 0.0
    matrix[1][3] = 0.0
    matrix[2][3] = 0.0
    matrix[3][3] = 1.0
    return matrix


if __name__ == "__main__":
    main()
