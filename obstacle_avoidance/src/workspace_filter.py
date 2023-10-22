#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
import numpy as np
from sensor_msgs import point_cloud2
import tf2_ros
import tf2_sensor_msgs
from tf2_sensor_msgs import tf2_sensor_msgs
import tf2_geometry_msgs
import pcl
import std_msgs
import time

class WorkspaceFilter:
    def __init__(self, base_boundaries,target_frame):
        rospy.init_node('workspace_filter')

        self.base_boundaries = base_boundaries
        self.target_frame = target_frame
        self.tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        self.cloud_sub = rospy.Subscriber('/camera/depth/color/points_sampled', PointCloud2, self.pointcloud_callback,buff_size=10000)

        # Create a publisher for the filtered point cloud
        self.filtered_cloud_pub = rospy.Publisher('/camera/depth/color/points_bounded', PointCloud2, queue_size=1)


    def pointcloud_callback(self, cloud_msg):
        # Transform point cloud to the desired frame
        #start = time.time()
        cloud_transformed_msg = self.transform_pointcloud(cloud_msg,target_frame)
        
        cloud_transformed = list(point_cloud2.read_points(cloud_transformed_msg, field_names=("x", "y", "z"), skip_nans=True))
        
        cloud_transformed = np.array(cloud_transformed)
        # Filter points outside the specified box area
        filtered_cloud = self.filter_pointcloud(cloud_transformed)
        
        # Publish the filtered point cloud
        self.publish_filtered_cloud(filtered_cloud, cloud_transformed_msg.header)
        #print(time.time() - start)
        #print('---')

    def transform_pointcloud(self, cloud_msg, target_frame):
        if not cloud_msg.header.frame_id == target_frame:
            try:
                # Look up the transformation from the source frame to the target frame
                transform = self.tf_buffer.lookup_transform(target_frame, cloud_msg.header.frame_id, rospy.Time(0), rospy.Duration(0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logwarn("Transform not available!")
                return

            # Apply the transformation to the PointCloud
            transformed_pcl_pc = tf2_sensor_msgs.do_transform_cloud(cloud_msg, transform)
            transformed_pcl_pc.header.stamp=cloud_msg.header.stamp
        else:
            transformed_pcl_pc = cloud_msg

        return transformed_pcl_pc
    
    def filter_pointcloud(self, cloud):
        x_min, x_max, y_min, y_max, z_min, z_max = base_boundaries

        # Filter points outside the specified box area
        mask_x = np.logical_and(cloud[:, 0] >= x_min, cloud[:, 0] <= x_max)
        mask_y = np.logical_and(cloud[:, 1] >= y_min, cloud[:, 1] <= y_max)
        mask_z = np.logical_and(cloud[:, 2] >= z_min, cloud[:, 2] <= z_max)

        # Combine masks to get the final filter
        final_mask = np.logical_and(np.logical_and(mask_x, mask_y), mask_z)

        # Apply the filter to the point cloud
        filtered_cloud = cloud[final_mask]

        return filtered_cloud
    
    def publish_filtered_cloud(self, cloud,header):
        # Create a PointCloud2 message
        filtered_cloud_msg = point_cloud2.create_cloud_xyz32(header, cloud)

        # Publish the filtered point cloud
        self.filtered_cloud_pub.publish(filtered_cloud_msg)

if __name__ == '__main__':
    base_boundaries = [-0.45,0.45,-0.15,0.75,0.05,100] # x_min, x_max, y_min, y_max, z_min, z_max
    target_frame = 'base_link'
    transformer = WorkspaceFilter(base_boundaries, target_frame)

    rospy.spin()
