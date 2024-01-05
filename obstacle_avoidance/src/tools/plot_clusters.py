import pcl
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.colors as mcolors

if __name__ == "__main__":

    cloud = pcl.load('/home/izan/src/obstacle_avoidance/data/input_cloud_1.pcd')

    cluster_tolerance = 0.01
    min_cluster_size = 1
    max_cluster_size = 25000
    cluster_size_th = 50

    tree = cloud.make_kdtree()
    ec = cloud.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(cluster_tolerance)
    ec.set_MinClusterSize(min_cluster_size)
    ec.set_MaxClusterSize(max_cluster_size)
    ec.set_SearchMethod(tree)
    cluster_indices = ec.Extract()


    clusters = []
    centroids = []
    outliers = []
    for i, indices in enumerate(cluster_indices):
        points = []
        for j,index in enumerate(indices):
            points.append([cloud[index][0],cloud[index][1],cloud[index][2]])
        
        if (len(indices) >= cluster_size_th):
            
            centroid = np.mean(np.array(points), axis=0)
            
            centroids.append(centroid)
            clusters.append(np.array(points))
        else:
            outliers+=points
    


    fig = plt.figure(figsize=(18,12))
    ax = fig.add_subplot(projection='3d')
    linewidth = 2.0
    radius = 30
    fontsize = 18
    labelpad=20

    colors = ['tab:blue', 'tab:orange', 'tab:green', 'tab:purple', 'tab:brown', 'tab:pink', 'tab:gray', 'tab:olive', 'tab:cyan']
    
    if len(centroids)>0:
        centroids = np.array(centroids)
        ax.scatter(centroids[:,0],centroids[:,1],centroids[:,2], s=radius*3, color='tab:red', label='Centroid', alpha=1.0)

    for i,cluster in enumerate(clusters):
        ax.scatter(cluster[:,0],cluster[:,1],cluster[:,2], s=radius, color=colors[i], label='Cluster '+str(i), alpha=0.5)

    if len(outliers)>0:
        outliers = np.array(outliers)
        ax.scatter(outliers[:,0],outliers[:,1],outliers[:,2], s=radius, color='k', label='Cluster None', alpha=0.5)
 
    ax.set_xlim([0.3,-0.3])
    ax.set_ylim([0.9,0.3])
    ax.set_zlim([-0.3,0.3])
    ax.view_init(elev=60, azim=-46)
    ax.dist = 12
    ax.tick_params(labelsize=fontsize)
    ax.set_xlabel('X(m)', fontsize=fontsize*1.5, labelpad=labelpad*1.5)
    ax.set_ylabel('Y(m)', fontsize=fontsize*1.5, labelpad=labelpad*1.5)
    ax.set_zlabel('Z(m)', fontsize=fontsize*1.5, labelpad=labelpad*1.5)
    ax.xaxis.pane.fill = False
    ax.yaxis.pane.fill = False
    ax.zaxis.pane.fill = False
    #ax.invert_yaxis() # Match robot base axis
    ax.legend(fontsize=fontsize, loc="center right", bbox_to_anchor=(1.2, 0.5))
    box = ax.get_position()
    ax.set_position([box.x0+0.06, box.y0, box.width, box.height])
    ax.set_title("Euclidean clustering", fontsize=fontsize*2, y=1.0)
    
    plt.tight_layout()
    plt.show()
        
    