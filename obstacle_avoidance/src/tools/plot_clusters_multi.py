import pcl
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.colors as mcolors

if __name__ == "__main__":

    cloud = pcl.load('/home/izan/.ros/input_cloud.pcd')

    
    cluster_tolerance_list = [0.02, 0.03, 0.04 ,0.05]
    min_cluster_size = 1
    max_cluster_size = 25000
    cluster_size_th = 50

    #fig = plt.figure(figsize=(18,12))
    fig = plt.figure(figsize=(6*len(cluster_tolerance_list),7))

    legend_dict={}
    for c,cluster_tolerance in enumerate(cluster_tolerance_list):
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

        ax = plt.subplot(1,len(cluster_tolerance_list), c+1, projection='3d')
        #ax = fig.add_subplot(projection='3d')
        
        linewidth = 2.0
        radius = 15
        fontsize = 15
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
        ax.set_zlim([-0.2,0.4])
        ax.view_init(elev=30, azim=-40)
        ax.dist = 13
        ax.tick_params(labelsize=fontsize*0.9)
        ax.set_xlabel('X(m)', fontsize=fontsize*1.2, labelpad=labelpad*1.5)
        ax.set_ylabel('Y(m)', fontsize=fontsize*1.2, labelpad=labelpad*1.5)
        ax.set_zlabel('Z(m)', fontsize=fontsize*1.2, labelpad=labelpad*1.5)
        ax.xaxis.pane.fill = False
        ax.yaxis.pane.fill = False
        ax.zaxis.pane.fill = False

        ax.set_title("Tolerance="+str(cluster_tolerance), fontsize=fontsize*1.5, y=0.95)

        handles, labels = fig.gca().get_legend_handles_labels()
        legend_dict.update(dict(zip(labels, handles)))

    #ax.invert_yaxis() # Match robot base axis
    
    legend = fig.legend(legend_dict.values(), legend_dict.keys(),ncol=len(legend_dict.keys()),fontsize=fontsize, loc="lower center", bbox_to_anchor=(0.5, 0.0))
    
    for l in range(len(legend.legendHandles)):
        legend.legendHandles[l]._sizes = [100]
    box = ax.get_position()
    #fig.subplots_adjust(left=0.1, right=0.9, bottom=0.2, top=0.8, wspace=0.5, hspace=0.3)
    #ax.set_position([box.x0+0.06, box.y0, box.width, box.height])
    fig.suptitle("Euclidean clustering", fontsize=fontsize*2, y=1.0)
    
    plt.tight_layout()
    plt.show()
        
    