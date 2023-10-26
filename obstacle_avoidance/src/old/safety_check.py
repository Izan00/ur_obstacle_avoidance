#!/usr/bin/env python3

import rospy

from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose

import numpy as np
import ros_numpy

from sklearn import metrics
from sklearn.cluster import DBSCAN, KMeans

import time

from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
import pcl

import open3d as o3d
import pyvista as pv
import pymeshfix

from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Bool

import sys
import moveit_commander
from moveit_msgs.msg import CollisionObject
from moveit_msgs.msg import PlanningScene, ObjectColor
from geometry_msgs.msg import Pose, PoseStamped
from shape_msgs.msg import SolidPrimitive, Mesh

from geometry_msgs.msg import Point, Quaternion, Vector3
from std_msgs.msg import ColorRGBA

class SafetyCheck:
    def __init__(self, min_samples,eps,cluster_size_th, voxel_size, alpha, algorithm, leaf_size, delayluna_method):
        rospy.init_node('safety_check')

        rospy.Subscriber("/camera/depth/color/points_bounded", PointCloud2, self.pointCloudCallback, buff_size=100000)
        self.safe_stop_publisher = rospy.Publisher('/robot_safe_stop', Bool, queue_size=1)
        self.centroid_marker_publisher = rospy.Publisher('/centroids_marker', Marker, queue_size=1)
        self.collision_scene_marker_pub = rospy.Publisher('collision_object_marker', PlanningScene, queue_size=1)

        moveit_commander.roscpp_initialize(sys.argv)
        self.scene = moveit_commander.PlanningSceneInterface()
        #robot = moveit_commander.RobotCommander()
        #group_name = "manipulator"  # Replace with your planning group
        #group = moveit_commander.MoveGroupCommander(group_name)

        self.prev_centroids = []
        self.still_count = 0
        self.still = True
        self.movement = 0

        #Cluster params
        self.min_samples = min_samples
        self.eps = eps
        self.cluster_size_th = cluster_size_th
        self.leaf_size = leaf_size
        self.algorithm = algorithm

        self.delayluna_method = delayluna_method
        self.voxel_size = voxel_size
        self.alpha = alpha

    def create_clusters(self, cloud):
        #kmeans = KMeans(n_clusters=2, random_state=0, n_init="auto").fit(X)
        #print(kmeans.cluster_centers_)
        
        db = DBSCAN(eps=self.eps, min_samples=self.min_samples, algorithm=self.algorithm, leaf_size=self.leaf_size,n_jobs=-1).fit(cloud)
        labels = db.labels_
        
        clusters = []
        centroids = []
        for i,label in enumerate(np.unique(labels)):
                cluster_indices = np.where(labels == label)[0]
                cluster_points = cloud[cluster_indices]
                if cluster_points.shape[0]>=self.cluster_size_th:
                    centroid = np.mean(cluster_points, axis=0)
                    centroids.append(centroid)
                    clusters.append(cluster_points)

        return centroids, clusters

    def add_collision_object(self, object_name, header):
        object_pose = Pose()
        object_pose.position = Point(0,0,0)
        object_pose.orientation = Quaternion(0,0,0,1)

        collision_object = CollisionObject()
        collision_object.header = header
        collision_object.id = object_name

        object_pose_stamped = PoseStamped()
        object_pose_stamped.header = header
        object_pose_stamped.pose = object_pose

        collision_object.meshes.append(object_name+'.stl')
        collision_object.mesh_poses.append(object_pose_stamped.pose)
  
        collision_object.operation = CollisionObject.ADD

        self.scene.add_mesh(object_name, object_pose_stamped, object_name+'.stl')
        
        # Publish collision object marker for visualization in RViz
        planning_scene = PlanningScene()
        planning_scene.world.collision_objects.append(collision_object)
        planning_scene.is_diff = True
        self.collision_scene_marker_pub.publish(planning_scene)


    def pointCloudCallback(self, cloud_msg):
        start = time.time()
        start1 = time.time()
        cloud = list(point_cloud2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True))
        print(time.time() - start1)
        if len(cloud) > 0:
            cloud = np.array(cloud)
            #np.save('point_cloud.npy', cloud)
            start1 = time.time()
            centroids, clusters = self.create_clusters(cloud)
            print('Cluster time:',time.time() - start1)
            #print('clusters:',len(clusters))
            print('------')
            for i, (centroid,cluster) in enumerate(zip(centroids, clusters)):
                cluster_name = 'cluster_'+str(i)
                
                start1 = time.time()
                self.publish_centroid_marker(centroids, cloud_msg.header)
                print('Publish time:',time.time() - start1)
                print('------')
                
                start1 = time.time()
                #self.movement_check(centroids)
                print('Movement check time:',time.time() - start1)

                start1 = time.time()
                # Surface generation
                #self.create_mesh(cluster,cluster_name)
                print('Create mesh time:',time.time() - start1)
                # Add collision object to the planning scene
                start1 = time.time()
                #self.add_collision_object(cluster_name, cloud_msg.header)
                print('Add collsion time:',time.time() - start1)
                
                
            print('Total time:',time.time() - start)
            print('=====')
                #cluster_pcl2 = array_to_pointcloud(cluster_points)
                #cluster_msg = ros_numpy.msgify(PointCloud2, cluster_pcl2, stamp=pcl_msg.header.stamp, frame_id='base_link')
                #cluster_publisher = rospy.Publisher('/clustered_pointcloud_'+str(i), PointCloud2, queue_size=1)
                #cluster_publisher.publish(cluster_msg)


    def create_mesh(self, points, centroid_name):
        #print(points.shape)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        #o3d.visualization.draw_geometries([pcd])
        pcd = pcd.voxel_down_sample(voxel_size=self.voxel_size)
        #o3d.visualization.draw_geometries([pcd])

        points = np.asarray(pcd.points)
        #print(points.shape)
        point_cloud = pv.PolyData(points)
        
        #mesh = point_cloud.reconstruct_surface(nbr_sz=10,sample_spacing=0.01)
   
        if self.delayluna_method=='2d':
            mesh = point_cloud.delaunay_2d(alpha=self.alpha)

        elif self.delayluna_method=='3d':
            mesh = point_cloud.delaunay_3d(alpha=self.alpha, progress_bar=False).extract_surface().triangulate()

        #volume = point_cloud.delaunay_3d(alpha=self.alpha)
        #mesh = volume.extract_geometry()
        
        mesh.clean(inplace=True)
        mesh.compute_normals(inplace=True)
        
        #mesh.subdivide(1, subfilter="linear", inplace=True)
     
        #fixer = pymeshfix.MeshFix(mesh)
        #fixer.repair(joincomp=True, remove_smallest_components=False)
        #mesh = fixer.mesh 
        
        #mesh.flip_normal([1.0, 1.0, -1.0], transform_all_input_vectors=True, inplace=True)

        mesh.save(centroid_name+'.stl')


    def publish_centroid_marker(self, centroids, header):
        marker = Marker()
        marker.header = header
        marker.type = Marker.SPHERE_LIST  # Set marker type to MESH_RESOURCE
        marker.action = Marker.ADD
        marker.scale = Vector3(0.1,0.1,0.1)
        marker.lifetime = rospy.Duration(0)
        marker.id = 0
        marker.color = ColorRGBA(1.0,0.0,0.0,1.0 )#r,g,b,a
        marker.pose.position = Point(0,0,0)
        marker.pose.orientation = Quaternion(0,0,0,1)

        marker.points = [Point(x=centroid[0], y=centroid[1], z=centroid[2]) for centroid in centroids]
        
        self.centroid_marker_publisher.publish(marker)

    def movement_check(self, centroids):

        if (self.prev_centroids == [] and centroids != []) or (self.prev_centroids != [] and centroids == []):
            self.movement+=np.inf
        elif self.prev_centroids == [] and centroids == []:
            self.movement+=0
        else:
            for prev_centroid in self.prev_centroids:
                dist = np.min(np.linalg.norm(np.array(centroids) - prev_centroid, axis=1))
                self.movement+=dist
                print('Dist: ',dist)
        print('Movement:',self.movement)

        self.prev_centroids = centroids

        # Check if workspace is still
        if self.still_count>5:
            if abs(self.movement)<0.05:
                self.still = True
            else:
                self.still= False
            self.still_count = 0
            self.movement = 0
        else:
            self.still_count += 1

        print('Still:', self.still)

        safe_stop_msg = Bool()

        if self.still:
            safe_stop_msg.data = False
        else:
            safe_stop_msg.data = True
        
        self.safe_stop_publisher.publish(safe_stop_msg)
        
        #TODO 
        ''' 1.tigger save workspace centroid state before robot move
            2.check state diff during robot move
            3.Stop robot while state change, 
            4.when movement stop, compare state with 1. save
            5.if same resume move, else ask for path check again
        ''' 
        
def array_to_pointcloud(pc_array, intensity=1.0):
    pc_msg = np.zeros(pc_array.shape[0], dtype=[
        ('x', np.float32),
        ('y', np.float32),
        ('z', np.float32),
        ('intensity', np.float32),
    ])
    if pc_array.shape[0] != 0:
        pc_msg['x'] = pc_array[:, 0]
        pc_msg['y'] = pc_array[:, 1]
        pc_msg['z'] = pc_array[:, 2]
        pc_msg['intensity'] = np.ones(pc_array.shape[0])*intensity

    return pc_msg

if __name__ == '__main__':
    
    min_samples = 1
    eps = 0.05
    algorithm=['auto', 'ball_tree', 'kd_tree', 'brute'][0]
    leaf_size=30
    cluster_size_th = 50
    delayluna_method=['2d','3d'][0]
    voxel_size = 0.01
    alpha = 0.05

    safety_check = SafetyCheck(min_samples, eps, cluster_size_th, voxel_size, alpha, algorithm, leaf_size,delayluna_method)
    
    rospy.spin()
