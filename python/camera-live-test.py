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
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.rgb8, 30)

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
    clipping_distance_in_meters = 4 # 4 meter
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

        #clusteringDepthImage(pointcloud)


        if geom_added == False:
            vis.add_geometry(pointcloud)
            geom_added = True

        vis.update_geometry(pointcloud)
        vis.poll_events()
        vis.update_renderer()



        process_time = datetime.now() - dt0
        print("FPS: " + str(1 / process_time.total_seconds()))

    pipeline.stop()
    cv2.destroyAllWindows()
    vis.destroy_window()
    del vis


def clusteringDepthImage(point_cloud):
    with o3d.utility.VerbosityContextManager(
            o3d.utility.VerbosityLevel.Debug) as cm:
        labels = np.array(
            point_cloud.cluster_dbscan(eps=0.02, min_points=10, print_progress=True))

    max_label = labels.max()
    print(f"point cloud has {max_label + 1} clusters")
    colors = point_cloud.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))

    #colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
    #colors[labels < 0] = 0
    # pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
    # o3d.visualization.draw_geometries([point_cloud],
    #                                   zoom=0.455,
    #                                   front=[-0.4999, -0.1659, -0.8499],
    #                                   lookat=[2.1813, 2.0619, 2.0999],
    #                                   up=[0.1204, -0.9852, 0.1215])

    # print(depth_image.shape)
    #
    # for y in range(0, depth_image.shape[0]):
    #     for x in range(0, depth_image.shape[1]):
    #         pixel = [y,x]
    #         point = rs.rs2_deproject_pixel_to_point(intrinsics, pixel,depth_image[y,x])
    #         #print(point)
    #
    # X = np.array([[1, 2], [1, 4], [1, 0],
    #               ...[10, 2], [10, 4], [10, 0]])
    # pass


if __name__ == "__main__":
    main()
