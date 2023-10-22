#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/crop_box.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vector>

class WorkspaceFilter
{
public:
    WorkspaceFilter(const std::vector<double>& base_boundaries,const std::string& target_frame)
    : base_boundaries(base_boundaries), target_frame(target_frame)
    {
        ros::NodeHandle nh;
        tf_listener_.reset(new tf2_ros::TransformListener(tf_buffer_));

        cloud_sub_ = nh.subscribe("/camera/depth/color/points_sampled", 1, &WorkspaceFilter::pointcloudCallback, this);
        filtered_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/camera/depth/color/points_bounded", 1);
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

        filtered_cloud_pub_.publish(filtered_cloud_msg);
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
        double x_min = base_boundaries[0];
        double x_max = base_boundaries[1];
        double y_min = base_boundaries[2];
        double y_max = base_boundaries[3];
        double z_min = base_boundaries[4];
        double z_max = base_boundaries[5];

        pcl::PointCloud<pcl::PointXYZ> filtered_cloud;

        pcl::CropBox<pcl::PointXYZ> crop_filter;
        crop_filter.setInputCloud(cloud.makeShared());
        crop_filter.setMin(Eigen::Vector4f(x_min, y_min, z_min, 1.0));
        crop_filter.setMax(Eigen::Vector4f(x_max, y_max, z_max, 1.0));
        crop_filter.filter(filtered_cloud);

        return filtered_cloud;
    };

private:
    std::vector<double> base_boundaries;
    std::string target_frame;
    ros::Subscriber cloud_sub_;
    ros::Publisher filtered_cloud_pub_;
    tf2_ros::Buffer tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv,"workspace_filter_2");

    std::vector<double> base_boundaries = {-0.45,0.45,-0.15,0.75,0.05,100}; // x_min, x_max, y_min, y_max, z_min, z_max
    std::string target_frame = "base_link";
    WorkspaceFilter workspace_filter(base_boundaries, target_frame);

    ros::spin();

    return 0;
};
