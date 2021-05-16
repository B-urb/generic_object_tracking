pc=rs.pointcloud()
pc.map_to(aligned_depth_frame)
points = pc.calculate(aligned_depth_frame)
pcl_points=pcl.PointCloud()
point_to_pcl(pcl_points,points)

vox = plc_msg.make_voxel_grid_filter()
LEAF_SIZE = 0.01
# Set the voxel (or leaf) size
vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
downsampled = vox.filter()


# PassThrough Filter
 passthrough = outliers_removed.make_passthrough_filter()
 # Assign axis and range to the passthrough filter object.
 filter_axis = 'z'
 passthrough.set_filter_field_name(filter_axis)
 axis_min = 0.6
 axis_max = 1.1
 passthrough.set_filter_limits(axis_min, axis_max)
 passed = passthrough.filter()
 # Limiting on the Y axis too to avoid having the bins recognized as snacks
 passthrough = passed.make_passthrough_filter()
 # Assign axis and range to the passthrough filter object.
 filter_axis = 'y'
 passthrough.set_filter_field_name(filter_axis)
 axis_min = -0.45
 axis_max = +0.45
 passthrough.set_filter_limits(axis_min, axis_max)
 passed = passthrough.filter()

# RANSAC Plane Segmentation
seg = passed.make_segmenter()
seg.set_model_type(pcl.SACMODEL_PLANE)
seg.set_method_type(pcl.SAC_RANSAC)
max_distance = LEAF_SIZE
seg.set_distance_threshold(max_distance)
inliers, coefficients = seg.segment()

# Extract inliers and outliers
# Extract inliers - tabletop
cloud_table = cloud_filtered.extract(inliers, negative=False)
# Extract outliers - objects
cloud_objects = cloud_filtered.extract(inliers, negative=True)


def do_statistical_outlier_filtering(pcl_data, mean_k, tresh):
    '''
    :param pcl_data: point could data subscriber
    :param mean_k:  number of neighboring points to analyze for any given point (10)
    :param tresh:   Any point with a mean distance larger than global will be considered outlier (0.001)
    :return: Statistical outlier filtered point cloud data
    '''
    outlier_filter = pcl_data.make_statistical_outlier_filter()
    outlier_filter.set_mean_k(mean_k)
    outlier_filter.set_std_dev_mul_thresh(tresh)
    return outlier_filter.filter()


 # Euclidean Clustering
 white_cloud = XYZRGB_to_XYZ(cloud_objects)
 tree = white_cloud.make_kdtree()
 # Create a cluster extraction object
 ec = white_cloud.make_EuclideanClusterExtraction()
 # Set tolerances for distance threshold
 # as well as minimum and maximum cluster size (in points)
 ec.set_ClusterTolerance(LEAF_SIZE*2)
 ec.set_MinClusterSize(10)
 ec.set_MaxClusterSize(2500)
 # Search the k-d tree for clusters
 ec.set_SearchMethod(tree)
 # Extract indices for each of the discovered clusters
 cluster_indices = ec.Extract()

