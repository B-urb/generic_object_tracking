import pyrealsense2 as rs
import numpy as np
import cv2
import open3d as o3d
from floodfill import Grid
import scipy
import matplotlib.pyplot as plt
import time
from tracking import processObjects

NUMBER_OF_OBJECTS = 100
MAX_DIST = 0.5
config_perception = {
    "IMAGE_WIDTH":640,
          "IMAGE_HEIGHT":480,
          "samplesX":32,
          "samplesY":32,
          "pruneThreshold":0.8,
          "greedThreshold":0.01,
          "mergeThreshold":0.1}


def main():
    # objects = np.empty(NUMBER_OF_OBJECTS, dtype=object)
    objects = []
    old_time = time.time()
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter('./data/output.avi', fourcc, 30.0, (1280, 720))
    pipeline = rs.pipeline()
    queue = rs.frame_queue(5000, keep_frames=True)
    config = rs.config()

    vis = o3d.visualization.Visualizer()
    vis.create_window('PCD', width=1280, height=720)
    pointcloud = o3d.geometry.PointCloud()
    vis.add_geometry(pointcloud)
    geom_added = False

    # note: using 640 x 480 depth resolution produces smooth depth boundaries
    #       using rs.format.bgr8 for color image format for OpenCV based image visualization
    config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.rgb8, 30)

    config.enable_device_from_file("./data/realsense.bag", repeat_playback=True)

    # Start streaming
    profile = pipeline.start(config, queue)
    depth_sensor = profile.get_device().first_depth_sensor()

    # Getting the depth sensor's depth scale (see rs-align example for explanation)
    depth_scale = depth_sensor.get_depth_scale()

    # We will not display the background of objects more than
    #  clipping_distance_in_meters meters away
    clipping_distance_in_meters = 4  # 3 meter
    clipping_distance = clipping_distance_in_meters / depth_scale

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

    transformation_matrix = [[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]]
    inv_transform = np.linalg.inv(transformation_matrix)
    #grid = Grid(config_perception)
    #grid.initialize()
    # Streaming loop
    frame_count = 0
    intrinsics = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
    pcd_old = o3d.geometry.PointCloud()

    while frame_count < 1000:

        cycle_time_start = time.time()

        # Get frameset of color and depth
        # frames = pipeline.wait_for_frames()
        frames = queue.wait_for_frame().as_frameset()
        current_time = frames.get_frame_metadata(rs.frame_metadata_value.time_of_arrival)

        start = time.time()
        # Align the depth frame to color frame
        aligned_frames = align.process(frames)

        # Get aligned frames
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()


        # Validate that both frames are valid
        if not depth_frame or not color_frame:
            continue

        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        end = time.time()
        #print("Frame Post Processing took: " + str((end - start) * 1000) + "ms")

        #############
        ####
        ## Create Point Cloud
        ####
        #############

        start = time.time()
        img_depth = o3d.geometry.Image(depth_image)
        img_color = o3d.geometry.Image(color_image)
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            img_color, img_depth, depth_trunc=clipping_distance_in_meters, convert_rgb_to_intensity=False)

        pinhole_camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(intrinsics.width, intrinsics.height, intrinsics.fx,
                                                                     intrinsics.fy,
                                                                     intrinsics.ppx, intrinsics.ppy)
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, pinhole_camera_intrinsic)
        pcd = pcd.remove_non_finite_points(remove_nan=True ,remove_infinite=True)
        pcd = pcd.voxel_down_sample(voxel_size=0.05)
        points = np.asarray(pcd.points)
        pcd_tree = o3d.geometry.KDTreeFlann(pcd)
        start = time.time()
        [k, idx, _] = pcd_tree.search_knn_vector_3d(pcd.points[1500], 4)
        end = time.time()
        print("Neighbour search: " + str((end - start) * 1000) + "ms")
        print(idx)

        #grid.update(points)



if __name__ == "__main__":
    main()
