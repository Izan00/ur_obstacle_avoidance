#include <ros/ros.h>

#include "obstacle_avoidance.h"


class SafetyCheck
{
public:
    SafetyCheck(ros::NodeHandle& nh) :nh_(nh)
    {
        // Topics
        std::string cloud_sub_topic;
        std::string safe_stop_pub_topic;
        std::string centroids_marker_pub_topic;
        
        // Parameters update
        nh_.param<int>("min_cluster_size", min_cluster_size, 1);
        nh_.param<int>("max_samples", max_cluster_size, 25000);
        nh_.param<double>("eps", eps, 0.05);
        nh_.param<int>("cluster_size_th", cluster_size_th, 50);
        nh_.param<bool>("save_cloud", save_cloud, false);
        nh_.param<bool>("save_mesh", save_mesh, false);
        nh_.param<double>("alpha", alpha, 0.0);
        nh_.param<std::string>("mesh_algo", mesh_algo, "d3d");
        nh_.param<std::string>("cloud_sub_topic", cloud_sub_topic, "/camera/depth/color/points_filtered");
        nh_.param<std::string>("safe_stop_pub_topic", safe_stop_pub_topic, "/robot_safe_stop");
        nh_.param<std::string>("centroids_marker_pub_topic", centroids_marker_pub_topic, "/centroids_marker");
        
        // Ros publishers/subscribers
        pcl_sub = nh_.subscribe(cloud_sub_topic, 1, &SafetyCheck::pointCloudCallback, this);
        safe_stop_pub = nh_.advertise<std_msgs::Bool>(safe_stop_pub_topic, 1);
        centroid_marker_pub = nh_.advertise<visualization_msgs::Marker>(centroids_marker_pub_topic, 1);
        
        // Set safety check variables
        still_count = 0;
        still = true;
        movement = 0;
    };

    void createClusters(const pcl::PointCloud<pcl::PointXYZ>& cloud, std::vector<pcl::PointCloud<pcl::PointXYZ>>& clusters, std::vector<Eigen::Vector4f>& centroids)
    {
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud.makeShared());

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(eps);
        ec.setMinClusterSize(min_cluster_size);
        ec.setMaxClusterSize(max_cluster_size); // Set according to your needs
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud.makeShared());
        ec.extract(cluster_indices);

        for (const auto& indices : cluster_indices)
        {
            std::size_t cluster_sz_th = static_cast<std::size_t>(cluster_size_th);

            if (indices.indices.size() >= cluster_sz_th)
            {
                pcl::PointCloud<pcl::PointXYZ> cluster;
                pcl::copyPointCloud(cloud, indices, cluster);

                // Compute centroid
                Eigen::Vector4f centroid;
                pcl::compute3DCentroid(cluster, centroid);
                clusters.push_back(cluster);
                centroids.push_back(centroid);
            }
        }
    };

    void addCollisionObject(const std::string& object_name,const shape_msgs::Mesh& mesh, const std_msgs::Header& header)
    {
        // Create CollisionObject
        moveit_msgs::CollisionObject collision_object;
        collision_object.header = header;
        collision_object.id = object_name;
        collision_object.meshes.push_back(mesh);
        collision_object.pose = Pose::ZeroPose();

        // Update the collision object in the MoveIt! planning scene
        planning_scene_interface.applyCollisionObject(collision_object);
    };

    void createMesh(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_cloud, shape_msgs::Mesh& mesh_msg,const std::string save_name="out_mesh")
    {
        // Create vtkPoints and add points from the point cloud
        vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
        for (const auto& point : pcl_cloud->points)
        {
            points->InsertNextPoint(point.x, point.y, point.z);
        }

        // Create vtkPolyData and set the points
        vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
        polydata->SetPoints(points);

        vtkSmartPointer<vtkDelaunay2D> delaunay = vtkSmartPointer<vtkDelaunay2D>::New();
        vtkSmartPointer<vtkDelaunay3D> delaunay3D = vtkSmartPointer<vtkDelaunay3D>::New();
        vtkSmartPointer<vtkDataSetSurfaceFilter> surfaceFilter = vtkSmartPointer<vtkDataSetSurfaceFilter>::New();
        
        if(mesh_algo == "d2d")
        {
            // Perform Delaunay triangulation
            delaunay->SetInputData(polydata);
            delaunay->SetAlpha(alpha);
            // Extract the surface of the triangulation
            surfaceFilter->SetInputConnection(delaunay->GetOutputPort());
        }
        else if(mesh_algo == "d3d")
        {
            // Perform Delaunay triangulation
            delaunay3D->SetInputData(polydata);
            delaunay3D->SetAlpha(alpha);
            // Extract the surface of the triangulation
            surfaceFilter->SetInputConnection(delaunay3D->GetOutputPort());
        }

        // Clean the polydata
        vtkSmartPointer<vtkCleanPolyData> cleanPolyData = vtkSmartPointer<vtkCleanPolyData>::New();
        cleanPolyData->SetInputConnection(surfaceFilter->GetOutputPort());

        // Avoid segmentation fault before mesh conversion
        cleanPolyData->Update();

        // Extract vertices
        vtkPoints* vtkVertices = cleanPolyData->GetOutput()->GetPoints();
        for (vtkIdType i = 0; i < vtkVertices->GetNumberOfPoints(); ++i)
        {
            geometry_msgs::Point point;
            double* coords = vtkVertices->GetPoint(i);
            point = Point::PointXYZ(coords[0],coords[1],coords[2]);
            mesh_msg.vertices.push_back(point);
        }

        // Extract polygons
        vtkCellArray* vtkPolygons = cleanPolyData->GetOutput()->GetPolys();
        vtkPolygons->InitTraversal();
        vtkIdType npts, *pts;
        while (vtkPolygons->GetNextCell(npts, pts))
        {
            shape_msgs::MeshTriangle triangle;
            triangle.vertex_indices = Point::VertexID(pts[0],pts[1],pts[2]);
            mesh_msg.triangles.push_back(triangle);
        }

        // Save the reconstructed mesh file
        if(save_mesh==true)
        {
            std::string name = save_name +".stl";
            vtkSmartPointer<vtkSTLWriter> writer = vtkSmartPointer<vtkSTLWriter>::New();
            writer->SetFileName(name.c_str());
            writer->SetInputConnection(cleanPolyData->GetOutputPort());
            writer->Write();
        }

    };

    void publishCentroidMarker(const std::vector<Eigen::Vector4f>& centroids, const std_msgs::Header& header)
    {
        visualization_msgs::Marker marker;
        marker.header = header;
        marker.type = visualization_msgs::Marker::SPHERE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale = Point::VectorXYZ(0.1,0.1,0.1);
        marker.lifetime = ros::Duration(0);
        marker.id = 0;
        marker.color = Color::RGBA(1,0,0,1);
        marker.pose = Pose::ZeroPose();

        for (const auto& centroid : centroids)
        {
            geometry_msgs::Point point;
            point = Point::PointXYZ(centroid[0],centroid[1],centroid[2]);
            marker.points.push_back(point);
        }

        centroid_marker_pub.publish(marker);
    };

    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
    {
        // Convert cloud to pcl format
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*cloud_msg, cloud);

        std::vector<std::string> current_collisions;

		std::vector<pcl::PointCloud<pcl::PointXYZ>> clusters;
        std::vector<Eigen::Vector4f> centroids;
        
        // Comput centroids when cloud has points
        if (!cloud.empty())
        {
           createClusters(cloud, clusters, centroids);
        }
        else
        {
		   centroids.clear();
		}

        //Publish centroids in Rviz
        publishCentroidMarker(centroids, cloud_msg->header);
        
        // Check object movement
        movementCheck(centroids);
        
        // Process clusters
        for (size_t i = 0; i < centroids.size(); ++i)
        {
            std::string cluster_name = "cluster_" + std::to_string(i);
            
            shape_msgs::Mesh mesh;
            pcl::PointCloud<pcl::PointXYZ>::Ptr clusterPtr;
            clusterPtr = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(clusters[i]);
            
            // Save the point cloud to PCD file (ASCII format)
            if(save_cloud)
            {  
                pcl::io::savePCDFileASCII(cluster_name+".pcd", *clusterPtr);
            }

            // Create mesh from cluster
            createMesh(clusterPtr, mesh, cluster_name);
            
            // Update collision objec in planning scene
            addCollisionObject(cluster_name,mesh, cloud_msg->header);

            if(prev_collisions.size()>0)
            {
                prev_collisions.erase(prev_collisions.begin());
            }
            current_collisions.push_back(cluster_name);
        }

        // Remove collision objects from previos iteration
        if(prev_collisions.size()>0)
        {
            planning_scene_interface.removeCollisionObjects(prev_collisions);
            prev_collisions = current_collisions;
        }   
    };
    

    void movementCheck(const std::vector<Eigen::Vector4f>& centroids)
    {
        if ((prev_centroids.empty() && !centroids.empty()) || (!prev_centroids.empty() && centroids.empty()))
        {
            movement += std::numeric_limits<double>::infinity();
        }
        else if (prev_centroids.empty() && centroids.empty())
        {
            movement += 0;
        }
        else
        {
            for (const auto& prev_centroid : prev_centroids)
            {
                double dist = std::numeric_limits<double>::max();
                for (const auto& centroid : centroids)
                {
                    double temp_dist = std::sqrt(std::pow(prev_centroid[0] - centroid[0], 2) +
                                                 std::pow(prev_centroid[1] - centroid[1], 2) +
                                                 std::pow(prev_centroid[2] - centroid[2], 2));
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
    ros::Subscriber pcl_sub;
    ros::Publisher safe_stop_pub;
    ros::Publisher centroid_marker_pub;

    // Clsutering
    bool save_cloud;
    int min_cluster_size;
    int max_cluster_size;
    double eps;
    int cluster_size_th;

    // Mesh reconstruction
    std::string mesh_algo; // d3d, d2d
    bool save_mesh;
    double alpha;
    
    // Movement check
    std::vector<Eigen::Vector4f> prev_centroids;
    int still_count;
    bool still;
    double movement;

    // Collisions
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    std::vector<std::string> prev_collisions;
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "safety_check_2");

    ros::NodeHandle nh;

    SafetyCheck safety_check(nh);

    ros::spin();

    return 0;
};
