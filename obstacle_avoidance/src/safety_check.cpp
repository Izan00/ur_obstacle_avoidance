#include <ros/ros.h>

#include "obstacle_avoidance.h"


class SafetyCheck
{
public:
    SafetyCheck(ros::NodeHandle& nh) :nh_(nh)
    {
        // Topics
        std::string safe_stop_pub_topic;
        std::string centroids_sub_topic;
        
        // Parameters update
        nh_.param<std::string>("centroids_sub_topic", centroids_sub_topic, "/centroids");
        nh_.param<std::string>("safe_stop_pub_topic", safe_stop_pub_topic, "/robot_safe_stop");
        
        // Ros publishers/subscribers
        pcl_sub = nh_.subscribe(cloud_sub_topic, 1, &SafetyCheck::centroidsCallback, this);
        
        // Set safety check variables
        still_count = 0;
        still = true;
        movement = 0;
    };


    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
    {
        // Convert cloud to pcl format
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*cloud_msg, cloud);

        movementCheck(cloud);
    };


    void movementCheck(const  pcl::PointCloud<pcl::PointXYZ>& centroids)
    {
        if ((prev_centroids.cloud.empty() && !centroids.cloud.empty()) || (!prev_centroids.cloud.empty() && centroids.cloud.empty()))
        {
            movement += std::numeric_limits<double>::infinity();
        }
        else if (prev_centroids.cloud.empty() && centroids.cloud.empty())
        {
            movement += 0;
        }
        else
        {
            for (const auto& prev_centroid : prev_centroids.cloud)
            {
                double dist = std::numeric_limits<double>::max();
                for (const auto& centroid : centroid.cloud)
                {
                    double temp_dist = std::sqrt(std::pow(prev_centroid.x - centroid.x, 2) +
                                                 std::pow(prev_centroid.y - centroid.y, 2) +
                                                 std::pow(prev_centroid.z - centroid.z, 2));
                    dist = std::min(dist, temp_dist);
                }
                movement += dist;
                std::cout << "Dist: " << dist << std::endl;
            }
        }

        std::cout << "Movement: " << movement << std::endl;

        prev_centroids = centroids;

        if (still_count > 5)
        {
            still = (std::abs(movement) < 0.05);
            still_count = 0;
            movement = 0;
        }
        else
        {
            still_count += 1;
        }

        std::cout << "Still: " << still << std::endl;

        std_msgs::Bool safe_stop_msg;
        safe_stop_msg.data = !still;
        safe_stop_pub.publish(safe_stop_msg);
    };

private:
    // Parameters handler
    ros::NodeHandle nh_;

    // Ros publishers/subscribers
    ros::Subscriber centroids_sub;
    ros::Publisher safe_stop_pub;
    
    // Movement check
    pcl::PointCloud<pcl::PointXYZ> prev_centroids;
    int still_count;
    bool still;
    double movement;
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "safety_check");

    ros::NodeHandle nh;

    SafetyCheck safety_check(nh);

    ros::spin();

    return 0;
};
