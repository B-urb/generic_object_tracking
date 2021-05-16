import pyrealsense2 as rs
import numpy as np
import cv2
import open3d as o3d
from datetime import datetime
import matplotlib.pyplot as plt
import time
from app.tracking_object import TrackingObject
from sklearn.cluster import KMeans


def main():
    #out = cv2.VideoWriter('./data/output.avi', -1, 20.0,(1280, 720))
    pipeline = rs.pipeline()
    config = rs.config()

    vis = o3d.visualization.Visualizer()
    vis.create_window('PCD', width=1280, height=720)
    pointcloud = o3d.geometry.KDTreeFlann()
    vis.add_geometry(pointcloud)
    geom_added = False

    # note: using 640 x 480 depth resolution produces smooth depth boundaries
    #       using rs.format.bgr8 for color image format for OpenCV based image visualization
    config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.rgb8, 30)

    config.enable_device_from_file("./data/realsense.bag", repeat_playback=True)

    # Start streaming
    profile = pipeline.start(config)
    depth_sensor = profile.get_device().first_depth_sensor()

    # Getting the depth sensor's depth scale (see rs-align example for explanation)
    depth_scale = depth_sensor.get_depth_scale()

    # We will not display the background of objects more than
    #  clipping_distance_in_meters meters away
    clipping_distance_in_meters = 3  # 3 meter
    clipping_distance = clipping_distance_in_meters / depth_scale

    # Create an align object
    # rs.align allows us to perform alignment of depth frames to others frames
    # The "align_to" is the stream type to which we plan to align depth frames.
    align_to = rs.stream.color
    align = rs.align(align_to)

    # Streaming loop
    frame_count = 0
    intrinsics = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
    pcd_old = o3d.geometry.PointCloud()

    while True:
        # Get frameset of color and depth
        frames = pipeline.wait_for_frames()

        # Align the depth frame to color frame
        aligned_frames = align.process(frames)

        # Get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        # Validate that both frames are valid
        if not aligned_depth_frame or not color_frame:
            continue

        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        img_depth = o3d.geometry.Image(depth_image)
        img_color = o3d.geometry.Image(color_image)
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            img_color, img_depth, depth_trunc=clipping_distance_in_meters, convert_rgb_to_intensity=False)

        pinhole_camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(intrinsics.width, intrinsics.height, intrinsics.fx,
                                                                     intrinsics.fy,
                                                                     intrinsics.ppx, intrinsics.ppy)
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, pinhole_camera_intrinsic)
        pcd_tree = o3d.geometry.KDTreeFlann(pcd)


        pointcloud.points = pcd_tree.points
        pointcloud.colors = pcd_tree.colors

        if geom_added == False:
            vis.add_geometry(pointcloud)
            geom_added = True

        #vis.capture_screen_image("./data/test.png")
        #screenshot = np.asanyarray(buf)
        #cv.ims
        #out.write(screenshot)


        vis.update_geometry(pointcloud)
        vis.poll_events()
        vis.update_renderer()
        frame_count += 1

    vis.destroy_window()
    del vis
    #out.release()
    pipeline.stop()


if __name__ == "__main__":
    main()
