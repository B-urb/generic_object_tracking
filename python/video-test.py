import pyrealsense2 as rs
import numpy as np
import cv2
import open3d as o3d
import scipy
from app.clustering import clusterDBscanO3D, clusterDBScanScipy, clusterMeanShift, clusterAgglomerative
import matplotlib.pyplot as plt
import time
from app.samplegrid import SampleGrid
from app.tracking import processObjects
from app.tracking_object import TrackingObject
from app.assignment import calc_cost_matrix, create_assignment


NUMBER_OF_OBJECTS = 100
MAX_DIST = 0.5
high_res_config = {
"IMAGE_WIDTH":1280,
          "IMAGE_HEIGHT":720,
          "samplesX":32,
          "samplesY":32,
            "maxDist":0.5,
            "NUMBER_OF_OBJECTS":100,
          "pruneThreshold":0.8,
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
    out = cv2.VideoWriter('./data/output.avi', fourcc, 30.0, (app_config["IMAGE_WIDTH"], app_config["IMAGE_HEIGHT"]))
    pipeline = rs.pipeline()
    queue = rs.frame_queue(5000, keep_frames=True)
    config = rs.config()

    vis = o3d.visualization.Visualizer()
    vis.create_window('PCD', width=app_config["IMAGE_WIDTH"], height=app_config["IMAGE_HEIGHT"])
    pointcloud = o3d.geometry.PointCloud()
    #vis.run()

    samplegrid = SampleGrid(app_config)
    samplegrid.initialize()

    #vis.add_geometry(pointcloud)
    geom_added = False

    # note: using 640 x 480 depth resolution produces smooth depth boundaries
    #       using rs.format.bgr8 for color image format for OpenCV based image visualization
    config.enable_stream(rs.stream.depth, app_config["IMAGE_WIDTH"], app_config["IMAGE_HEIGHT"], rs.format.z16, 30)
    config.enable_stream(rs.stream.color, app_config["IMAGE_WIDTH"], app_config["IMAGE_HEIGHT"], rs.format.rgb8, 30)

    #config.enable_record_to_file("./data/record_homeroom_high.bag")
    #config.enable_device_from_file("./data/realsense.bag", repeat_playback=True)
    config.enable_device_from_file("./data/record_homeroom_low.bag", repeat_playback=True)


    # Start streaming
    profile = pipeline.start(config, queue)
    depth_sensor = profile.get_device().first_depth_sensor()

    # Getting the depth sensor's depth scale (see rs-align example for explanation)
    depth_scale = depth_sensor.get_depth_scale()

    # We will not display the background of objects more than
    #  clipping_distance_in_meters meters away
    clipping_distance_in_meters = 3  # 3 meter
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

    # Streaming loop
    frame_count = 0
    intrinsics = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()

    while frame_count < 1500:

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

        if frame_count < 60:
            #print(frame_count)
            frame_count += 1
            continue
        # depth_frame = depth_to_disparity.process(depth_frame)
        # depth_frame = spatial.process(depth_frame)
        # depth_frame = temporal.process(depth_frame)
        # depth_frame = disparity_to_depth.process(depth_frame)
        # depth_frame = hole_filling.process(depth_frame)
        # depth_frame = decimation.process(depth_frame)

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

        # pc = rs.pointcloud()
        # points = pc.calculate(depth_frame)
        # point_array = np.asarray(points.get_vertices(2))
        # pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(point_array))
        #pcd = pcd.transform(transformation_matrix)

        #samplegrid.update(np.asarray(pcd.points))
        #samplegrid.findFloor()



        #voxels = grid.get_voxels()

        # query_point = np.asarray(pcd.points)[150]
        # voxel = grid.get_voxel(query_point)
        # voxel[2] = 4
        # vox = grid.get_voxel(voxel)


        pcd = pcd.remove_non_finite_points(remove_nan=True ,remove_infinite=True)




        # pc = rs.pointcloud()
        # points = pc.calculate(depth_frame)
        # point_array = np.asarray(points.get_vertices(2))
        # pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(point_array))
        # pcd = pcd.voxel_down_sample(0.05)
        # #print(pcd)


        pcd = pcd.voxel_down_sample(voxel_size=0.05)
        print(pcd)
        #pcd, indices = pcd.voxel_down_sample_and_trace(voxel_size=0.05, min_bound=[0,0,0], max_bound=[1280, 720,4500], approximate_class=False)
        #pcd = pcd.uniform_down_sample(every_k_points=4)
        end = time.time()
        #print("Pointcloud generation: " + str((end - start) * 1000) + "ms")

        ##################################################################################################
        ########
        ########
        ## Plane Segmentation and Downsampling
        ########
        #########
        start = time.time()
        plane_model, inliers = pcd.segment_plane(distance_threshold=0.04,
                                                 ransac_n=3,
                                                 num_iterations=200)
        [a, b, c, d] = plane_model

        pcd = pcd.select_by_index(inliers, invert=True)
        end = time.time()
        # print("Plane segmentation took: " + str((end - start) * 1000) + "ms")



        ######################################################################################################
        ########
        ########
        ## Clustering
        ########
        ########

        start = time.time()
        labels = clusterDBscanO3D(pcd)
        #labels = clusterDBScanScipy(np.asarray(pcd.points))
        #labels = clusterMeanShift(np.asarray(pcd.points))
        #labels = clusterAgglomerative(np.asarray(pcd.points))

        end = time.time()
        #print("Clustering took: " + str((end - start) * 1000) + "ms")
        max_label = labels.max()

        detections = []
        # deal with temporary loss, maybe keepm state(track of recent lost objects), add last seen counter
        for i in range(0, max_label + 1):
            detections.append(TrackingObject())

        point_array = np.asarray(pcd.points)
        for i, point in enumerate(point_array):
            cluster_id = labels[i]
            # print(cluster_id)
            if cluster_id != -1:
                detections[cluster_id].points.append(point)

        #print(f"point cloud has {max_label + 1} clusters")
        for obj in detections:
            if len(obj.points) < 60:  # or len(obj.points) > 2000:
                detections.remove(obj)

        for obj in detections:
            obj.calc_bounding_box(intrinsics)

        if len(objects) == 0:
            objects = detections
            old_time = frames.get_frame_metadata(rs.frame_metadata_value.time_of_arrival)


        print(f"There are {len(objects)} objects and {len(detections)} detections")
        start = time.time()
        cost_matrix = calc_cost_matrix(objects, detections)
        assignment_matrix = create_assignment(objects, cost_matrix, MAX_DIST)

        # TODO: Use kalman predict in case of occlusion, needs detected occlusion
        if len(assignment_matrix) > 0:
            for i, detection_index in enumerate(assignment_matrix):
                if assignment_matrix[i] != -1:
                    if objects[i].status == "candidate":
                        objects[i].state_counter += 1
                    if objects[i].status == "active":
                        objects[i].state_counter = 0
                    if objects[i].status == "lost":
                        objects[i].status = "active"

                    objects[i].points = detections[assignment_matrix[i]].points
                    objects[i].calc_bounding_box(intrinsics)
                    if frame_count % 30 ==0:
                        p1 = objects[i].last_position
                        p2 = objects[i].bounding_box.get_center()
                        objects[i].last_position = p2
                        p2 = detections[assignment_matrix[i]].bounding_box.get_center()
                        # pixel_1 = rs.rs2_project_point_to_pixel(intrinsics, p1.tolist())
                        # pixel_1 = [int(coord) for coord in pixel_1]
                        # pixel_2 = rs.rs2_project_point_to_pixel(intrinsics, p2.tolist())
                        # pixel_2 = [int(coord) for coord in pixel_2]
                        # distance_rs_1 = depth_frame.get_distance(pixel_1[0], pixel_1[1])
                        # distance_rs_2 = depth_frame.get_distance(pixel_2[0], pixel_2[1])
                        # point_1 = rs.rs2_deproject_pixel_to_point(intrinsics, pixel_1, distance_rs_1)
                        # point_2 = rs.rs2_deproject_pixel_to_point(intrinsics, pixel_2, distance_rs_2)
                        # distance_obj = scipy.spatial.distance.euclidean(point_1,
                        #                                                 point_2)  # sqrt(pow(upoint[0] - vpoint[0], 2) +pow(upoint[1] - vpoint[1], 2) + pow(upoint[2] - vpoint[2], 2))

                        distance = scipy.spatial.distance.euclidean(p1, p2)
                        time_elapsed = (current_time -  objects[i].last_measurement_timestamp)
                        velocity = calc_velocity(p1, p2, distance, time_elapsed)
                        objects[i].speed_vector = velocity
                        objects[i].last_measurement_timestamp = current_time
                        #old_time = current_time

                        # detections.remove(detections[j])
                else:
                    objects[i].state_counter += 1

        end = time.time()
        print("Assignment took: " + str((end - start) * 1000) + "ms")

        for i in range(len(detections)):
            if i not in assignment_matrix:
                candidate = detections[i]
                objects.append(candidate)
        #print("Time passed since last frame: " + str(time_elapsed) + "ms")
        #print("There are " + str(len(objects)) + " objects and " + str(len(detections)) + " candidates.")
        start = time.time()
        distances = pcd.compute_mahalanobis_distance()
        end = time.time()
        #print("Distance computation took: " + str((end - start) * 1000) + "ms")

        start = time.time()
        colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
        colors[labels < 0] = 0
        pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
        for obj in objects:
            if obj.status != "active":
                continue

            corners = np.asarray(obj.bounding_box.get_box_points())
            extent = obj.bounding_box.get_extent()
            pixels = []
            for point in corners:
                # TODO: transform this back, has to be extended to 4D coords
                #point = np.multiply(transformation_matrix,point.reshape(4,))
                pixel = rs.rs2_project_point_to_pixel(intrinsics, point.tolist())
                pixel = [int(coord) for coord in pixel]
                pixels.append(pixel)

            cv2.rectangle(color_image, tuple(pixels[1]), tuple(pixels[2]), (0, 255, 0), 2)


            label = "Obj-ID: "+str(obj.id) +"\n" \
                    + "vx: " + str(round(obj.speed_vector[0],2))  + " m/s \n"  \
                    + "vy: " + str(round(obj.speed_vector[1], 2)) + " m/s \n" \
                    + "vz: " + str(round(obj.speed_vector[2], 2))+ " m/s \n" \
                    + "speed  " + str(round(obj.speed_vector[3],2)) + "m/s \n"
            position = tuple(pixels[5])
            y0,dy = position[1], 12
            for i, line in enumerate(label.split('\n')):
                y = y0 + i * dy
                #cv2.putText(img, line, (50, y), cv2.FONT_HERSHEY_SIMPLEX, 1, 2)
                cv2.putText(color_image, line,(position[0]+10,y) , cv2.FONT_HERSHEY_COMPLEX, 0.5, (255, 255, 0), 1)


            end = time.time()
            #print("2D projection time: " + str((end - start) * 1000) + "ms")

        cv2.putText(color_image, str(frame_count), (50, 50), cv2.FONT_HERSHEY_COMPLEX, 2, (255, 255, 0), 1)
        out_img = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)
        cv2.imshow('bgr', out_img)

        key = cv2.waitKey(1)
        if key == ord('q'):
            break


        ########
        ########
        ## Rendering
        ########
        ########
        #pcd = pcd.transform(transformation_matrix)
        start = time.time()
        pointcloud.points = pcd.points
        pointcloud.colors = pcd.colors
        if geom_added == False:
            vis.add_geometry(pointcloud)
            geom_added = True
        for obj in objects:
            if obj.status == "active":
                vis.add_geometry(obj.bounding_box)
        vis.update_geometry(pointcloud)

        vis.reset_view_point(False)
        vis.update_renderer()
        vis.poll_events()
        for obj in objects:
            if obj.status == "active":
                vis.remove_geometry(obj.bounding_box)
        frame_count += 1
        #vis.reset_view_point(False)
        #vis.poll_events()
        #vis.update_renderer()
        #


        end = time.time()

        #print("Rendering took: " + str((end - start) * 1000) + "ms")
        #objects processing
        objects = processObjects(objects)

        ####
        ## Write Frame to video
        ####
        out.write(out_img)

        cycle_time_end = time.time()
        print("Cycle time: " + str((cycle_time_end - cycle_time_start) * 1000) + "ms")


    vis.destroy_window()
    del vis
    out.release()
    pipeline.stop()


def calc_velocity(p1, p2, distance, time_elapsed):  # time_elapsed in ms
    vx = 0.0
    vy = 0.0
    vz = 0.0
    speed = 0.0
    if time_elapsed > 0:
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        dz = p2[2] - p1[2]
        d = np.sqrt(dx * dx + dy * dy + dz * dz)
        speed = (distance/ time_elapsed) * 1000
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
def truncate(f, n):
    '''Truncates/pads a float f to n decimal places without rounding'''
    s = '{}'.format(f)
    if 'e' in s or 'E' in s:
        return '{0:.{1}f}'.format(f, n)
    i, p, d = s.partition('.')
    return '.'.join([i, (d+'0'*n)[:n]])

def calc_point_distance(p1, p2):
    squared_dist = np.sum((p1 - p2) ** 2, axis=0)
    dist = np.sqrt(squared_dist)
    return dist


if __name__ == "__main__":
    main()
