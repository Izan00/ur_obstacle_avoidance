#include <ros/ros.h>

#include "obstacle_avoidance.h"

class WorkspaceFilter
{
public:
    WorkspaceFilter(ros::NodeHandle& nh) :nh_(nh)
    {
        // Topics
        std::string cloud_sub_topic;
        std::string cloud_pub_topic;
        std::string filter_volume_pub_topic;

        // Parameters update
        nh_.param<double>("x_min", x_min, -0.45);
        nh_.param<double>("x_max", x_max, 0.45);
        nh_.param<double>("y_min", y_min, -0.15);
        nh_.param<double>("y_max", y_max, 0.75);
        nh_.param<double>("z_min", z_min, 0.05);
        nh_.param<double>("z_max", z_max, 100.0);
        nh_.param<std::string>("cloud_sub_topic", cloud_sub_topic, "/camera/depth/color/points_sampled");
        nh_.param<std::string>("cloud_pub_topic", cloud_pub_topic, "/camera/depth/color/points_bounded");
        nh_.param<std::string>("filter_volume_pub_topic", filter_volume_pub_topic, "/filter_volume_marker");
        nh_.param<std::string>("target_frame", target_frame, "base_link");
        
        // TF tranfomrs
        tf_listener_.reset(new tf2_ros::TransformListener(tf_buffer_));

        // Ros publishers/subscribers
        cloud_sub = nh_.subscribe(cloud_sub_topic, 1, &WorkspaceFilter::pointcloudCallback, this);
        filtered_cloud_pub = nh_.advertise<sensor_msgs::PointCloud2>(cloud_pub_topic, 1);
        filter_volume_marker_pub = nh_.advertise<visualization_msgs::Marker>(filter_volume_pub_topic, 1);
        
        filter_volume_marker.header.frame_id = target_frame;
        filter_volume_marker.type = visualization_msgs::Marker::CUBE;
        filter_volume_marker.action = visualization_msgs::Marker::ADD;
        filter_volume_marker.scale = Point::VectorXYZ(abs(x_max-x_min),abs(y_max-y_min),abs(z_max-z_min));
        filter_volume_marker.lifetime = ros::Duration(0);
        filter_volume_marker.id = 0;
        filter_volume_marker.color = Color::RGBA(1,0,1,0.5);
        filter_volume_marker.pose.position.x = x_min + abs(x_max-x_min)/2;
        filter_volume_marker.pose.position.y = y_min + abs(y_max-y_min)/2;
        filter_volume_marker.pose.position.z = z_min + abs(z_max-z_min)/2;
        filter_volume_marker.pose.orientation.x = 0;
        filter_volume_marker.pose.orientation.y = 0;
        filter_volume_marker.pose.orientation.z = 0;
        filter_volume_marker.pose.orientation.w = 1;
    };
    
    void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg)
    {
        sensor_msgs::PointCloud2 cloud_transformed_msg = transformPointCloud(*cloud_msg);

        pcl::PointCloud<pcl::PointXYZ> cloud_transformed;
        pcl::fromROSMsg(cloud_transformed_msg, cloud_transformed);

        pcl::PointCloud<pcl::PointXYZ> filtered_cloud = filterPointCloud(cloud_transformed);

        sensor_msgs::PointCloud2 filtered_cloud_msg;
        pcl::toROSMsg(filtered_cloud, filtered_cloud_msg);
        filtered_cloud_msg.header = cloud_transformed_msg.header;

        filtered_cloud_pub.publish(filtered_cloud_msg);

        filter_volume_marker_pub.publish(filter_volume_marker);
    };

    sensor_msgs::PointCloud2 transformPointCloud(const sensor_msgs::PointCloud2 &cloud_msg)
    {
        if (cloud_msg.header.frame_id != target_frame)
        {
            try
            {
                geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(target_frame, cloud_msg.header.frame_id, ros::Time(0), ros::Duration(0));
                sensor_msgs::PointCloud2 transformed_pcl_pc;
                tf2::doTransform(cloud_msg, transformed_pcl_pc, transform);
                transformed_pcl_pc.header.stamp = cloud_msg.header.stamp;
                return transformed_pcl_pc;
            }
            catch (tf2::TransformException &ex)
            {
                ROS_WARN_STREAM("Transform not available! " << ex.what());
                return cloud_msg;
            }
        }
        else
        {
            return cloud_msg;
        }
    };

    pcl::PointCloud<pcl::PointXYZ> filterPointCloud(const pcl::PointCloud<pcl::PointXYZ> &cloud)
    {
        pcl::PointCloud<pcl::PointXYZ> filtered_cloud;
        pcl::CropBox<pcl::PointXYZ> crop_filter;
        crop_filter.setInputCloud(cloud.makeShared());
        crop_filter.setMin(Eigen::Vector4f(x_min, y_min, z_min, 1.0));
        crop_filter.setMax(Eigen::Vector4f(x_max, y_max, z_max, 1.0));
        crop_filter.filter(filtered_cloud);

        return filtered_cloud;
    };

private:
    ros::NodeHandle nh_;
    double x_min; 
    double x_max; 
    double y_min; 
    double y_max; 
    double z_min; 
    double z_max;
    std::string target_frame;
    ros::Subscriber cloud_sub;
    ros::Publisher filtered_cloud_pub;
    ros::Publisher filter_volume_marker_pub;
    visualization_msgs::Marker filter_volume_marker;
    tf2_ros::Buffer tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv,"workspace_filter");

    ros::NodeHandle nh("~");

    WorkspaceFilter workspace_filter(nh);

    ros::spin();

    return 0;
};
