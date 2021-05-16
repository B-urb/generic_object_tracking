import numpy as np
import sklearn.cluster
import cv2
import open3d as o3d





def clusterDBscanO3D(pcd):
    labels = np.array(
        pcd.cluster_dbscan(eps=0.2, min_points=25, print_progress=False))

    return labels


def clusterMeanShift(points):
    clustering = sklearn.cluster.MeanShift(bandwidth=1,bin_seeding=True,cluster_all=False,n_jobs=20,max_iter=50).fit(points)
    return clustering.labels_

    pass

def clusterDBScanScipy(points):
    clustering = sklearn.cluster.DBSCAN(eps=0.02,min_samples=20, metric='euclidean',n_jobs=-1).fit(points)
    return clustering.labels_

def clusterAgglomerative(points):
    clustering = sklearn.cluster.AgglomerativeClustering(n_clusters=None,linkage='average',distance_threshold=1).fit(points)
    return clustering.labels_

def clusterKMeans(points):
    clustering = sklearn.cluster.KMeans(n_clusters=2, random_state=0).fit(points)
    return clustering.labels_

