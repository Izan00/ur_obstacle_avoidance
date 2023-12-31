#include <ros/ros.h>

#include "obstacle_avoidance.h"


class CollisionVolumeReconstructuion
{
public:
    CollisionVolumeReconstructuion(ros::NodeHandle& nh) :nh_(nh)
    {
        // Topics
        std::string cloud_sub_topic;
        std::string centroids_pub_topic;
        std::string centroids_marker_pub_topic;
        
        // Parameters update
        nh_.param<int>("min_cluster_size", min_cluster_size, 1);
        nh_.param<int>("max_samples", max_cluster_size, 25000);
        nh_.param<double>("cluster_tolerance", cluster_tolerance, 0.05);
        nh_.param<int>("cluster_size_th", cluster_size_th, 50);
        nh_.param<bool>("save_input_cloud", save_input_cloud, false);
        nh_.param<bool>("save_cluster_cloud", save_cluster_cloud, false);
        nh_.param<bool>("save_mesh", save_mesh, false);
        nh_.param<double>("delaunay_alpha", delaunay_alpha, 0.0);
        nh_.param<double>("delaunay_tolerance", delaunay_tolerance, 0.0);
        nh_.param<double>("delaunay_offset", delaunay_offset, 0.0);
        nh_.param<double>("gaussian_radius", gaussian_radius, 0.03);
        nh_.param<int>("gaussian_dimensions", gaussian_dimensions, 50);
        nh_.param<int>("convex_hull_planes", convex_hull_planes, 5);
        nh_.param<double>("convex_hull_shrinkwrap_sphere_radius", convex_hull_shrinkwrap_sphere_radius, 1.0);
        nh_.param<int>("convex_hull_shrinkwrap_sphere_angle_resolution", convex_hull_shrinkwrap_sphere_angle_resolution, 50);
        nh_.param<double>("convex_hull_shrinkwrap_resolution", convex_hull_shrinkwrap_resolution, 0.04);
        nh_.param<std::string>("mesh_algo", mesh_algo, "d3d");
        nh_.param<std::string>("cloud_sub_topic", cloud_sub_topic, "/camera/depth/color/points_filtered");
        nh_.param<std::string>("centroids_pub_topic", centroids_pub_topic, "/centroids");
        nh_.param<std::string>("centroids_marker_pub_topic", centroids_marker_pub_topic, "/centroids_marker");
        
        // Ros publishers/subscribers
        pcl_sub = nh_.subscribe(cloud_sub_topic, 1, &CollisionVolumeReconstructuion::pointCloudCallback, this);
        centroids_pub = nh.advertise<sensor_msgs::PointCloud2>(centroids_pub_topic, 1);
        centroids_marker_pub = nh_.advertise<visualization_msgs::Marker>(centroids_marker_pub_topic, 1);
        
    };

    void createClusters(const pcl::PointCloud<pcl::PointXYZ>& cloud, std::vector<pcl::PointCloud<pcl::PointXYZ>>& clusters, std::vector<Eigen::Vector4f>& centroids)
    {
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud.makeShared());

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(cluster_tolerance);
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

    void addCollisionObject(const std::vector<shape_msgs::Mesh> meshes, const std_msgs::Header header, const std::string object_name="workspace_collsions")
    {
        // Create CollisionObject
        moveit_msgs::CollisionObject collision_object;
        collision_object.header = header;
        collision_object.id = object_name;
        collision_object.meshes=  meshes;
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
        vtkSmartPointer<vtkCleanPolyData> cleanPolyData = vtkSmartPointer<vtkCleanPolyData>::New();
        
        if(mesh_algo == "d2d")
        {
            // Perform Delaunay 2D triangulation
            delaunay->SetInputData(polydata);
            delaunay->SetAlpha(delaunay_alpha);
            // Extract the surface of the triangulation
            vtkSmartPointer<vtkDataSetSurfaceFilter> surfaceFilter = vtkSmartPointer<vtkDataSetSurfaceFilter>::New();
            surfaceFilter->SetInputConnection(delaunay->GetOutputPort());

            // Clean the polydata
            cleanPolyData->SetInputConnection(surfaceFilter->GetOutputPort());
        }
        else if(mesh_algo == "d3d")
        {
            // Perform Delaunay 3D triangulation
            delaunay3D->SetInputData(polydata);
            delaunay3D->SetAlpha(delaunay_alpha);
            //delaunay3D->SetTolerance(delaunay_tolerance);
            //delaunay3D->SetOffset(delaunay_offset);
            // Extract the surface of the triangulation
            vtkSmartPointer<vtkDataSetSurfaceFilter> surfaceFilter = vtkSmartPointer<vtkDataSetSurfaceFilter>::New();
            surfaceFilter->SetInputConnection(delaunay3D->GetOutputPort());
            // Clean the polydata
            cleanPolyData->SetInputConnection(surfaceFilter->GetOutputPort());
        }
        else if(mesh_algo == "gauss")
        {   
            //Perform Gaussian Splatter
            vtkSmartPointer<vtkGaussianSplatter> splatter = vtkSmartPointer<vtkGaussianSplatter>::New();;
            splatter->SetInputData(polydata);
            splatter->SetSampleDimensions(gaussian_dimensions,gaussian_dimensions,gaussian_dimensions);
            splatter->SetRadius(gaussian_radius);
            splatter->ScalarWarpingOff();
            // Extract the surface
            vtkSmartPointer<vtkContourFilter> surfaceFilter = vtkSmartPointer<vtkContourFilter>::New();
            surfaceFilter->SetInputConnection(splatter->GetOutputPort());
            surfaceFilter->SetValue(0, 0.01);
            // Clean the polydata
            cleanPolyData->SetInputConnection(surfaceFilter->GetOutputPort());
        }
        else if(mesh_algo == "unp")
        {
            // Perform Surface Reconstruction
            vtkSmartPointer<vtkSurfaceReconstructionFilter> surface = vtkSmartPointer<vtkSurfaceReconstructionFilter>::New();;
            surface->SetInputData(polydata);
            // Extract the surface
            vtkSmartPointer<vtkContourFilter> surfaceFilter = vtkSmartPointer<vtkContourFilter>::New();
            surfaceFilter->SetInputConnection(surface->GetOutputPort());
            surfaceFilter->SetValue(0, 0.0);
            // Rpair normals
            vtkSmartPointer<vtkReverseSense> reverse = vtkSmartPointer<vtkReverseSense>::New();
            reverse->SetInputConnection(surfaceFilter->GetOutputPort());
            reverse->ReverseCellsOn();
            reverse->ReverseNormalsOn();   
            // Clean the polydata
            cleanPolyData->SetInputConnection(reverse->GetOutputPort());
        }
        else if(mesh_algo == "ch")
        {
            vtkSmartPointer<vtkHull> hullFilter = vtkSmartPointer<vtkHull>::New();
            hullFilter->SetInputData(polydata);
            hullFilter->AddCubeFacePlanes();
            hullFilter->AddRecursiveSpherePlanes(convex_hull_planes);
            // Clean the polydata
            cleanPolyData->SetInputConnection(hullFilter->GetOutputPort());
        }
        
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

    void publishCentroidsMarker(const std::vector<Eigen::Vector4f>& centroids, const std_msgs::Header& header)
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

        centroids_marker_pub.publish(marker);
    };

    void publishCentroids(const std::vector<Eigen::Vector4f>& centroids, const std_msgs::Header& header)
    {
        // Convert the Eigen vector to PCL point cloud
        pcl::PointCloud<pcl::PointXYZ> pcl_centroids_cloud;
        pcl_centroids_cloud.points.resize(centroids.size());
        for (size_t i = 0; i < centroids.size(); ++i) {
            pcl_centroids_cloud.points[i].x = centroids[i](0);
            pcl_centroids_cloud.points[i].y = centroids[i](1);
            pcl_centroids_cloud.points[i].z = centroids[i](2);
        }

        // Convert PCL point cloud to ROS sensor_msgs::PointCloud2
        sensor_msgs::PointCloud2 centroids_cloud;
        pcl::toROSMsg(pcl_centroids_cloud, centroids_cloud);

        centroids_cloud.header = header;
        centroids_pub.publish(centroids_cloud);
    };


    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
    {
        // Convert cloud to pcl format
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*cloud_msg, cloud);

        // Save the point cloud to PCD file (ASCII format)

		std::vector<pcl::PointCloud<pcl::PointXYZ>> clusters;
        std::vector<Eigen::Vector4f> centroids;
        
        // Comput centroids when cloud has points
        if (!cloud.empty())
        {
            if(save_input_cloud)
                pcl::io::savePCDFileASCII("input_cloud.pcd", cloud);
            createClusters(cloud, clusters, centroids);
        }
        else
		   centroids.clear();
		

        // Publish centrids 
        publishCentroids(centroids, cloud_msg->header);

        // Publish centroids in Rviz
        //publishCentroidsMarker(centroids, cloud_msg->header);
        
        std::vector<shape_msgs::Mesh> meshes;
        std::vector<std::string> current_collisions;

        // Process clusters
        for (size_t i = 0; i < centroids.size(); ++i)
        {
            std::string cluster_name = "collision_cluster_" + std::to_string(i);
            
            shape_msgs::Mesh mesh;
            pcl::PointCloud<pcl::PointXYZ>::Ptr clusterPtr;
            clusterPtr = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(clusters[i]);
            
            // Save the point cloud to PCD file (ASCII format)
            if(save_cluster_cloud)
            {  
                pcl::io::savePCDFileASCII(cluster_name+".pcd", *clusterPtr);
            }

            // Create mesh from cluster
            auto start_time = std::chrono::high_resolution_clock::now();
            createMesh(clusterPtr, mesh, cluster_name);
            auto end_time = std::chrono::high_resolution_clock::now();
            double duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count()/1000.0;
            //std::cout <<"Mesh execution time: "<<duration<< std::endl;
           
            meshes.push_back(mesh);
            // Update collision objec in planning scene
            addCollisionObject(meshes, cloud_msg->header, cluster_name);
            
            meshes.clear(); // for single item variant

            current_collisions.push_back(cluster_name);
        }

        //addCollisionObject(meshes, cloud_msg->header, "workspace_collsions");

        // Remove old objects
        if(collisions.size()>current_collisions.size()){
            std::vector<std::string> difference;
            for (int i = collisions.size()-current_collisions.size(); i <collisions.size(); ++i)
            {
                difference.push_back(collisions[i]);
            }
            
            planning_scene_interface.removeCollisionObjects(difference);
        }

        // Remove collision objects from previos iteration if clusters are empty
        if(centroids.size()==0)
        {
            //std::vector<std::string> collisions;
            //collisions.push_back("workspace_collsions");
            planning_scene_interface.removeCollisionObjects(collisions);
        } 
        collisions.clear();
        collisions = current_collisions;
    };

private:
    // Parameters handler
    ros::NodeHandle nh_;

    // Ros publishers/subscribers
    ros::Subscriber pcl_sub;
    ros::Publisher centroids_pub;
    ros::Publisher centroids_marker_pub;

    // Clustering
    bool save_input_cloud;
    bool save_cluster_cloud;
    int min_cluster_size;
    int max_cluster_size;
    double cluster_tolerance;
    int cluster_size_th;

    // Mesh reconstruction
    std::string mesh_algo; // d3d, d2d
    bool save_mesh;
    double delaunay_alpha;
    double delaunay_tolerance;
    double delaunay_offset;
    double gaussian_radius;
    int gaussian_dimensions;
    int convex_hull_planes;
    double convex_hull_shrinkwrap_sphere_radius;
    int convex_hull_shrinkwrap_sphere_angle_resolution;
    double convex_hull_shrinkwrap_resolution;

    std::vector<std::string> collisions;

    // Collisions
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "collsion_volume_reconstruction");

    ros::NodeHandle nh("~");

    CollisionVolumeReconstructuion collsion_volume_reconstruction(nh);

    ros::spin();

    return 0;
};
