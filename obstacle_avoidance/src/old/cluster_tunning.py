import numpy as np
import ros_numpy

from sklearn import metrics
from sklearn.cluster import DBSCAN, KMeans

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def create_clusters(cloud):
    #kmeans = KMeans(n_clusters=2, random_state=0, n_init="auto").fit(X)
    #print(kmeans.cluster_centers_)
    
    db = DBSCAN(eps=0.05, min_samples=1, algorithm='auto', leaf_size=30).fit(cloud)
    labels = db.labels_
    
    clusters = []
    centroids = []
    for i,label in enumerate(np.unique(labels)):
            cluster_indices = np.where(labels == label)[0]
            cluster_points = cloud[cluster_indices]
            if cluster_points.shape[0]>=20:
                centroid = np.mean(cluster_points, axis=0)
                centroids.append(centroid)
                clusters.append(cluster_points)

    return centroids, clusters

def plot_clusters_3d(clusters, centroids=None):
    """
    Plot a list of clusters in 3D space with centroids.

    Parameters:
    clusters (list of arrays): Each array represents a cluster.
    centroids (list of arrays, optional): List of centroids corresponding to clusters.
    """
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    for i, cluster in enumerate(clusters):
        cluster = np.array(cluster)
        ax.scatter(cluster[:, 0], cluster[:, 1], cluster[:, 2], label='cluster_'+str(i))

        # Plot centroids if available
        if centroids is not None and len(centroids) > i:
            centroid = np.array(centroids[i])
            ax.scatter(centroid[0], centroid[1], centroid[2], marker='x', s=200, c='red', label='cluster_'+str(i))

    ax.set_title('3D Cluster Plot with Centroids')
    ax.set_xlabel('X-axis')
    ax.set_ylabel('Y-axis')
    ax.set_zlabel('Z-axis')
    ax.legend()
    plt.show()

def plot_clusters(clusters,centroids):
    """
    Plot a list of clusters.

    Parameters:
    clusters (list of arrays): Each array represents a cluster.
    """
    plt.figure(figsize=(8, 8))

    for i, cluster in enumerate(clusters):
        cluster = np.array(cluster)
        plt.scatter(cluster[:, 0], cluster[:, 1], label='cluster_'+str(i))

        # Plot centroids if available
        if centroids is not None and len(centroids) > i:
            centroid = np.array(centroids[i])
            plt.scatter(centroid[0], centroid[1], marker='x', s=200, c='red', label='cluster_'+str(i))

    plt.title('Cluster Plot')
    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    plt.legend()
    plt.show()

cloud = np.load('src/obstacle_avoidance/point_cloud.npy')

centroids, clusters = create_clusters(cloud)
print('clusters:',len(clusters))
#plot_clusters(clusters,centroids)
plot_clusters_3d(clusters,centroids)
    
        

       