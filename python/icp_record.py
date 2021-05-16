import pyrealsense2 as rs
import numpy as np
import cv2
import open3d as o3d
from datetime import datetime
from sklearn.cluster import KMeans


# Configure depth and color streams


def main():

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

    # Start streaming
    #queue =rs.frame_queue(50)
    cfg = pipeline.start(config)#, queue)
    align = rs.align(rs.stream.color)
    profile = cfg.get_stream(rs.stream.depth)
    vis = o3d.visualization.Visualizer()
    vis.create_window('PCD', width=1280, height=720)
    pointcloud = o3d.geometry.PointCloud()
    geom_added = False

    intrinsics = profile.as_video_stream_profile().get_intrinsics()
    print(intrinsics)

    depth_sensor = cfg.get_device().first_depth_sensor()

    depth_scale = depth_sensor.get_depth_scale()
    print("Depth Scale is: ", depth_scale)

    #  clipping_distance_in_meters meters away
    clipping_distance_in_meters = 3  # 1 meter
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
    frame_count = 1
    while True:
        dt0 = datetime.now()
        frames = pipeline.wait_for_frames()
        #frames = queue.wait_for_frame().as_frameset()
        frames = align.process(frames)
        profile = frames.get_profile()

        depth_frame = frames.get_depth_frame()
        # depth_frame = decimation.process(depth_frame)
        depth_frame = depth_to_disparity.process(depth_frame)
        depth_frame = spatial.process(depth_frame)
        depth_frame = temporal.process(depth_frame)
        depth_frame = disparity_to_depth.process(depth_frame)
        depth_frame = hole_filling.process(depth_frame)

        # We will be removing the background of objects more than

        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())



        img_depth = o3d.geometry.Image(depth_image)
        img_color = o3d.geometry.Image(color_image)
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            img_color, img_depth, depth_trunc=clipping_distance_in_meters, convert_rgb_to_intensity=False)

        pinhole_camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(intrinsics.width, intrinsics.height, intrinsics.fx,
                                                                     intrinsics.fy,
                                                                     intrinsics.ppx, intrinsics.ppy)
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, pinhole_camera_intrinsic)
        pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
        pointcloud.points = pcd.points
        pointcloud.colors = pcd.colors

        cv2.imshow('bgr', color_image)
        key = cv2.waitKey(1)
        if key == ord('q'):
           break

        if key == ord('c'):
            filename = "./data/"+ str(frame_count) + "_cloud.pcd"
            o3d.io.write_point_cloud(filename, pcd)
            print("captured")
            frame_count += 1

        if key == ord('n'):
            continue



        process_time = datetime.now() - dt0
        print("FPS: " + str(1 / process_time.total_seconds()))

    pipeline.stop()
    cv2.destroyAllWindows()
    vis.destroy_window()
    del vis


if __name__ == "__main__":
    main()
