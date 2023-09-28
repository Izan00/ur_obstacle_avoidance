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
import tf2_ros
import tf2_sensor_msgs
from tf2_sensor_msgs import tf2_sensor_msgs
import tf2_geometry_msgs
import pcl

import open3d as o3d
import pyvista as pv
import pymeshfix

from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Bool

tf_listener = None
saved_centroids = []
still_count = 0

remove_ground = True


def create_clusters(X):
    #kmeans = KMeans(n_clusters=2, random_state=0, n_init="auto").fit(X)
    #print(kmeans.cluster_centers_)
    
    db = DBSCAN(eps=0.01, min_samples=10).fit(X)
    labels = db.labels_

    # Number of clusters in labels, ignoring noise if present.
    n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
    #print("Estimated number of clusters: %d" % n_clusters_)

    return labels

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

def create_urdf():
    # Define the file path
    file_path = "/home/izan/ur_ws/src/obstacle_avoidance/models/sample.sdf"  # Replace with your file path

    # Read the file
    with open(file_path, "r") as file:
        file_content = file.read()

    # Modify the content by replacing the string
    file_content = file_content.replace('stl_path', '/home/izan/.ros/mesh.stl')
    file_content = file_content.replace('model_name', 'model_1')
    
    file_path = file_path.split('.')[0]+'_temp.'+file_path.split('.')[1]
    # Write the modified content back to the file
    with open(file_path, "w") as file:
        file.write(file_content)

    print("File modified and saved successfully.")

def load_urdf():
    # Define the path to your SDF model file (e.g., cube.sdf)
    model_path = "/home/izan/ur_ws/src/obstacle_avoidance/models/sample_temp.sdf"
    
    # Define the name of the model you want to spawn in Gazebo
    model_name = "model1"
    
    try:
        # Look up the transformation from the source frame to the target frame
        transform = tf_buffer.lookup_transform('wolrd', 'base_link', rospy.Time(0), rospy.Duration(1.0))
        print(type(transform))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.logwarn("Transform not available!")
        
    #TODO Fix mesh (.stl) origin
    #TODO Remove absolute dirs

    # Set the pose of the cube
    cube_pose = Pose()
    cube_pose.position.x = 0.0
    cube_pose.position.y = 0.0
    cube_pose.position.z = 1.0  # Adjust the height as needed
    
    # Load the model
    print('Wainting gazebo')
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    print('Spawning')
    try:
        spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        spawn_model(model_name, open(model_path, 'r').read(), "robot_namespace", cube_pose, "world")
        rospy.loginfo(f"Successfully spawned {model_name} in Gazebo!")
    except rospy.ServiceException as e:
        rospy.logerr(f"Failed to spawn {model_name} in Gazebo: {e}")

def create_mesh(points):
    #print(points.shape)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    o3d.visualization.draw_geometries([pcd])
    pcd = pcd.voxel_down_sample(voxel_size=0.01) #0.02
    o3d.visualization.draw_geometries([pcd])

    #radius = 3*0.01
    #bpa_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(pcd,o3d.utility.DoubleVector([radius, radius * 2]))

    points = np.asarray(pcd.points)
    #print(points.shape)
    point_cloud = pv.PolyData(points)
    
    #mesh = point_cloud.reconstruct_surface(nbr_sz=10,sample_spacing=0.01)
    #mesh.plot(show_edges=True)
    
    mesh = point_cloud.delaunay_2d(alpha=0.06).clean() #0.04
    mesh.plot(show_edges=True)

    #mesh = point_cloud.delaunay_3d(alpha=0.02, progress_bar=False).extract_surface().triangulate()#.clean()
    #mesh.plot(show_edges=True)

    mesh.compute_normals(inplace=True)
    
    #mesh.subdivide(1, subfilter="linear", inplace=True)
    '''
    fixer = pymeshfix.MeshFix(mesh)
    fixer.repair(joincomp=True, remove_smallest_components=False)
    fixer.mesh.plot()
    mesh = fixer.mesh 
    '''
    mesh.flip_normal([1.0, 1.0, -1.0], transform_all_input_vectors=True, inplace=True)

    mesh.save('mesh.stl')

    print('Mesh created')  
    
def publish_mesh_marker():
    marker = Marker()
    marker.header.frame_id = 'base_link'
    marker.header.stamp = rospy.Time.now() #msg.header.stamp
    marker.type = Marker.MESH_RESOURCE  # Set marker type to MESH_RESOURCE
    marker.action = Marker.ADD
    marker.scale.x = 1.0  # Set the scale of the mesh
    marker.scale.y = 1.0  # Set the scale of the mesh
    marker.scale.z = 1.0  # Set the scale of the mesh
    marker.lifetime = rospy.Duration()
    # Set the color (RGBA) of the marker
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0

    marker.mesh_resource = "file:///home/izan/.ros/mesh.stl"

    marker.pose.position.x = 0
    marker.pose.position.y = 0
    marker.pose.position.z = 0
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    # Create a publisher for the marker
    print('publishing mesh')
    # Publish the marker
    marker_mesh_publisher.publish(marker)
    for _ in range(100):
        marker_mesh_publisher.publish(marker)
        #print('publish')

def publish_centroid_marker(centroids):
    marker_array = MarkerArray() 
    
    for id,centroid in enumerate(centroids):
        
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.header.stamp = rospy.Time.now() #msg.header.stamp
        marker.type = Marker.SPHERE  # Set marker type to MESH_RESOURCE
        marker.action = Marker.ADD
        marker.scale.x = 0.1  # Set the scale of the mesh
        marker.scale.y = 0.1  # Set the scale of the mesh
        marker.scale.z = 0.1  # Set the scale of the mesh
        marker.lifetime = rospy.Duration()
        marker.id = id
        # Set the color (RGBA) of the marker
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.pose.position.x = centroid[0]
        marker.pose.position.y = centroid[1]
        marker.pose.position.z = centroid[2]
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker_array.markers.append(marker)

    # Publish the marker
    if len(marker_array.markers)>0:
        marker_centroid_publisher.publish(marker_array)

def movement_check(centroids):
    global still_count
    movement = 0
    for saved_centroid in saved_centroids:
        movement+=np.min(np.linalg.norm(np.array(centroids) - saved_centroid, axis=1))
    print('Movement:',movement)

    # Check if workspace is still
    if movement<0.1:
        still_count += 1
    else: 
        still_count = 0

    safe_stop_msg = Bool()

    # Check if workspace is still for some time
    if still_count == 5:
        saved_centroid = centroids
    elif still_count > 5:
        safe_stop_msg.data = False
    else:
        safe_stop_msg.data = True
    
    safe_stop_publisher.publish(safe_stop_msg)
    
    #TODO 
    ''' 1.tigger save workspace centroid state before robot move
        2.check state diff during robot move
        3.Stop robot while state change, 
        4.when movement stop, compare state with 1. save
        5.if same resume move, else ask for path check again
    ''' 
    
def pointCloudCallback(pcl_msg):
    if not pcl_msg.header.frame_id == 'base_link':
        try:
            # Look up the transformation from the source frame to the target frame
            transform = tf_buffer.lookup_transform('base_link', pcl_msg.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Transform not available!")
            return

        # Apply the transformation to the PointCloud
        transformed_pcl_pc = tf2_sensor_msgs.do_transform_cloud(pcl_msg, transform)
    else:
        transformed_pcl_pc = pcl_msg

    #sor = transformed_pcl_pc.make_voxel_grid_filter()
    #sor.set_leaf_size(0.01, 0.01, 0.01)
    #filtered_pcl_pc= sor.filter()

    pc_list = []
    pcg_list = []
    for point in point_cloud2.read_points(transformed_pcl_pc, skip_nans=True):
        #print(point[0],point[1],point[2])
        if not (remove_ground and point[2]<=0.01):
            pc_list.append( [point[0],point[1],point[2]] )
        else:
            pcg_list.append( [point[0],point[1],point[2]] )
    
    if len(pc_list) > 0:

        pc_array = np.array(pc_list)
        pcg_array = np.array(pcg_list)

        pc_pcl2 = array_to_pointcloud(pc_array)

        no_ground_msg = ros_numpy.msgify(PointCloud2, pc_pcl2, stamp=pcl_msg.header.stamp, frame_id='base_link')
        #no_ground_publisher.publish(no_ground_msg)

        cluster_labels = create_clusters(pc_array)

        centroids = []

        for i,label in enumerate(np.unique(cluster_labels)):
            cluster_indices = np.where(cluster_labels == label)[0]
            # Extract the cluster points
            cluster_points = pc_array[cluster_indices]
            if cluster_points.shape[0]>=100: #TODO define threshold as var
                #print(cluster_points.shape)
                
                centroid = np.mean(cluster_points, axis=0)
                centroids.append(centroid)
                
                # Surface publish
                #create_mesh(cluster_points)
                #publish_mesh_marker()
                #create_urdf()
                #load_urdf()

            movement_check(centroids)
            publish_centroid_marker(centroids)

            #cluster_pcl2 = array_to_pointcloud(cluster_points)
            #cluster_msg = ros_numpy.msgify(PointCloud2, cluster_pcl2, stamp=pcl_msg.header.stamp, frame_id='base_link')
            #cluster_publisher = rospy.Publisher('/clustered_pointcloud_'+str(i), PointCloud2, queue_size=1)
            #cluster_publisher.publish(cluster_msg)

if __name__ == '__main__':
    
    rospy.init_node('safety_check')

    rospy.Subscriber("/camera/depth/color/points_filtered", PointCloud2, pointCloudCallback, buff_size=1)
    #no_ground_publisher = rospy.Publisher('/no_ground_pointcloud', PointCloud2, queue_size=1)
    marker_mesh_publisher = rospy.Publisher('/cluster_marker', Marker, queue_size=1)
    marker_centroid_publisher = rospy.Publisher('/centroids_marker', MarkerArray, queue_size=1)

    safe_stop_publisher = rospy.Publisher('/robot_safe_stop', Bool, queue_size=1)

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    rospy.spin()