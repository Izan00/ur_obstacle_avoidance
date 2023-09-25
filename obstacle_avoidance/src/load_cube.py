#!/usr/bin/env python3

import rospy
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose
import numpy as np
from sklearn import metrics
from sklearn.cluster import DBSCAN, KMeans


from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import sensor_msgs.point_cloud2
import ros_numpy
import time

import rospy
import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import tf2_geometry_msgs
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
import pcl
from pcl import PointCloud

def load_cube():
    # Define the path to your SDF model file (e.g., cube.sdf)
    model_path = rospy.get_param("~model_path", "package://obstacle_avoidance/models/cube.sdf")
    
    # Define the name of the model you want to spawn in Gazebo
    model_name = rospy.get_param("~model_name", "my_cube")
    
    # Set the pose of the cube
    cube_pose = Pose()
    cube_pose.position.x = 0.2
    cube_pose.position.y = 0.0
    cube_pose.position.z = 0.2  # Adjust the height as needed
    
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

def create_clusters(X):
    #kmeans = KMeans(n_clusters=2, random_state=0, n_init="auto").fit(X)
    #print(kmeans.cluster_centers_)
    
    db = DBSCAN(eps=0.01, min_samples=10).fit(X)
    labels = db.labels_

    # Number of clusters in labels, ignoring noise if present.
    n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)

    print("Estimated number of clusters: %d" % n_clusters_)

    return labels

def array_to_pointcloud(pc_array, intensity=1.0):
    pc_msg = np.zeros(pc_array.shape[0], dtype=[
        ('x', np.float32),
        ('y', np.float32),
        ('z', np.float32),
        ('intensity', np.float32),
    ])
    pc_msg['x'] = pc_array[:, 0]
    pc_msg['y'] = pc_array[:, 1]
    pc_msg['z'] = pc_array[:, 2]
    pc_msg['intensity'] = np.ones(pc_array.shape[0])*intensity

    return pc_msg

    
def pointCloudCallback(msg):
    try:
        # Look up the transformation from the source frame to the target frame
        transform = tf_buffer.lookup_transform('base_link', msg.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.logwarn("Transform not available!")
        return

    # Apply the transformation to the PointCloud
    transformed_pcl_pc = do_transform_cloud(msg, transform)
    
    pc_list = []
    pcg_list = []
    for point in sensor_msgs.point_cloud2.read_points(transformed_pcl_pc, skip_nans=True):
        #print(point[0],point[1],point[2])
        if not (remove_ground and point[2]<=0.01):
            pc_list.append( [point[0],point[1],point[2]] )
        else:
            pcg_list.append( [point[0],point[1],point[2]] )
    
    print(len(pc_list))
    print(len(pcg_list))

    pc_array = np.array(pc_list)
    pcg_array = np.array(pcg_list)

    pc_pcl2 = array_to_pointcloud(pc_array)
    no_ground_msg = ros_numpy.msgify(PointCloud2, pc_pcl2, stamp=msg.header.stamp, frame_id='base_link')
    no_ground_publisher.publish(no_ground_msg)

    cluster_labels = create_clusters(pc_array)

    for _ in range(10):
        for i,label in enumerate(np.unique(cluster_labels)):
            cluster_indices = np.where(cluster_labels == label)[0]
            # Extract the cluster points
            cluster_points = pc_array[cluster_indices]
            cluster_pcl2 = array_to_pointcloud(cluster_points)
            cluster_msg = ros_numpy.msgify(PointCloud2, cluster_pcl2, stamp=msg.header.stamp, frame_id='base_link')
            cluster_publisher = rospy.Publisher('/clustered_pointcloud_'+str(i), PointCloud2, queue_size=1)
            cluster_publisher.publish(cluster_msg)

    #print('msg published')

    #time.sleep(100000000)
    #exit()
    #print('pointcloud received')

rospy.init_node('load_cube')
rospy.Subscriber("/camera/depth/color/points", PointCloud2, pointCloudCallback)
no_ground_publisher = rospy.Publisher('/no_ground_pointcloud', PointCloud2, queue_size=1)

try:
    load_cube()
except:
    pass

remove_ground = True

tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)

rospy.spin()