#include <ros/ros.h>

#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/DeleteModel.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <std_msgs/Bool.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <shape_msgs/SolidPrimitive.h>
#include <shape_msgs/Mesh.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>

#include <pcl/conversions.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/mls.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/marching_cubes_hoppe.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/io/obj_io.h>
#include <pcl/io/pcd_io.h>

#include <pcl/surface/marching_cubes_hoppe.h>
#include <pcl/surface/marching_cubes_rbf.h>
#include <pcl/common/copy_point.h>
#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/surface/poisson.h>

class SafetyCheck
{
public:
    SafetyCheck(const int& min_samples, const double& eps, const int& cluster_size_th, const double& voxel_size, const double& alpha, const int& leaf_size,const std::string& mesh_algo)
        : min_samples(min_samples), eps(eps), cluster_size_th(cluster_size_th), voxel_size(voxel_size), alpha(alpha), leaf_size(leaf_size), mesh_algo(mesh_algo)
    {
        pcl_sub = nh.subscribe("/camera/depth/color/points_bounded", 1, &SafetyCheck::pointCloudCallback, this);
        safe_stop_pub = nh.advertise<std_msgs::Bool>("/robot_safe_stop", 1);
        centroid_marker_pub = nh.advertise<visualization_msgs::Marker>("/centroids_marker", 1);
        collision_scene_marker_pub = nh.advertise<moveit_msgs::PlanningScene>("collision_object_marker", 1);

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
        ec.setMinClusterSize(min_samples);
        ec.setMaxClusterSize(25000); // Set according to your needs
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

        geometry_msgs::Pose object_pose;
        object_pose.position.x = 0;
        object_pose.position.y = 0;
        object_pose.position.z = 0;
        object_pose.orientation.x = 0;
        object_pose.orientation.y = 0;
        object_pose.orientation.z = 0;
        object_pose.orientation.w = 1;

        // Update the collision object in the MoveIt! planning scene
        planning_scene_interface.applyCollisionObject(collision_object);
        //prev_collisions.push_back(object_name);
    };

    void createMesh(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_cloud, shape_msgs::Mesh& mesh_msg)
    {
 
        // Save the point cloud to a PCD file (ASCII format)
        pcl::io::savePCDFileASCII("output_cloud.pcd", *pcl_cloud);

        // Apply statistical outlier removal filter to clean the point cloud
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(pcl_cloud);
        sor.setMeanK(50);  // Adjust this value based on your point cloud characteristics
        sor.setStddevMulThresh(1.0);
        sor.filter(*pcl_cloud);

        pcl::PointCloud<pcl::PointNormal>::Ptr pcl_cloud_n(new pcl::PointCloud<pcl::PointNormal>);
        
        // Smooth the point cloud using Moving Least Squares (MLS) for better normals
        //pcl::PointCloud<pcl::PointNormal>::Ptr mls_points(new pcl::PointCloud<pcl::PointNormal>);
        //pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
        //mls.setInputCloud(pcl_cloud);
        //mls.setSearchRadius(0.02); // Adjust this value based on your point cloud characteristics
        //mls.process(*mls_points);

        // Compute normals
        // Create a pcl::PointCloud<pcl::Normal> to store the normals
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        // Create a NormalEstimation object
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        ne.setInputCloud(pcl_cloud);
        // Create a search tree (optional, but recommended)
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        ne.setSearchMethod(tree);
        // Use all neighbors in a sphere of radius 0.1m
        ne.setRadiusSearch(0.02);
        // Compute normals
        ne.compute(*normals);

        pcl::concatenateFields (*pcl_cloud, *normals, *pcl_cloud_n);

        if(mesh_algo == "concave_hull"){
            // Perform convex hull to create a mesh
            pcl::PointCloud<pcl::PointNormal>::Ptr out_cloud(new pcl::PointCloud<pcl::PointNormal>);
            pcl::ConcaveHull<pcl::PointNormal> concave_hull;
            concave_hull.setInputCloud(pcl_cloud_n);
            concave_hull.setAlpha (0.1);
            concave_hull.reconstruct(*out_cloud);
        }
        if(mesh_algo == "convex_hull"){
            // Perform convex hull to create a mesh
            pcl::PointCloud<pcl::PointNormal>::Ptr out_cloud(new pcl::PointCloud<pcl::PointNormal>);
            pcl::ConvexHull<pcl::PointNormal> convex_hull;
            convex_hull.setInputCloud(pcl_cloud_n);
            convex_hull.reconstruct(*out_cloud);

            // Convert PCL PointCloud to ROS shape_msgs::Mesh
            mesh_msg.vertices.resize(out_cloud->size());
            mesh_msg.triangles.resize((out_cloud->size() - 2) * 2);

            // Fill in vertices
            for (size_t i = 0; i < out_cloud->size(); ++i)
            {
                mesh_msg.vertices[i].x = out_cloud->points[i].x;
                mesh_msg.vertices[i].y = out_cloud->points[i].y;
                mesh_msg.vertices[i].z = out_cloud->points[i].z;
            }

            // Fill in triangles
            for (size_t i = 0; i < out_cloud->size() - 2; ++i)
            {
                mesh_msg.triangles[i].vertex_indices[0] = 0;
                mesh_msg.triangles[i].vertex_indices[1] = i + 1;
                mesh_msg.triangles[i].vertex_indices[2] = i + 2;
            }
        }
        else if(mesh_algo == "gp3")
        {
            // Create search tree*
            pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
            tree2->setInputCloud (pcl_cloud_n);

            // Initialize objects
            pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
            pcl::PolygonMesh mesh;

            // Set the maximum distance between connected points (maximum edge length)
            gp3.setSearchRadius (0.025);

            // Set typical values for the parameters
            gp3.setMu (5);
            gp3.setMaximumNearestNeighbors (200);
            gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
            gp3.setMinimumAngle(M_PI/18); // 10 degrees
            gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
            gp3.setNormalConsistency(true);

            // Get result
            gp3.setInputCloud (pcl_cloud_n);
            gp3.setSearchMethod (tree2);
            gp3.reconstruct (mesh); 

            pcl::io::saveOBJFile("output_mesh.obj", mesh);

            // Populate the vertices
            mesh_msg.vertices.resize(mesh.cloud.width * mesh.cloud.height);
            for (size_t i = 0; i < mesh.cloud.width * mesh.cloud.height; ++i) {
                geometry_msgs::Point point;
                point.x = mesh.cloud.data[i * mesh.cloud.point_step + mesh.cloud.fields[0].offset];
                point.y = mesh.cloud.data[i * mesh.cloud.point_step + mesh.cloud.fields[1].offset];
                point.z = mesh.cloud.data[i * mesh.cloud.point_step + mesh.cloud.fields[2].offset];
                mesh_msg.vertices[i] = point;
            }

            // Populate the triangles
            mesh_msg.triangles.resize(mesh.polygons.size());
            for (size_t i = 0; i < mesh.polygons.size(); ++i) {
                shape_msgs::MeshTriangle triangle;
                triangle.vertex_indices[0] = mesh.polygons[i].vertices[0];
                triangle.vertex_indices[1] = mesh.polygons[i].vertices[1];
                triangle.vertex_indices[2] = mesh.polygons[i].vertices[2];
                mesh_msg.triangles[i] = triangle;
            }
            std::cout<<mesh_msg.triangles.size()<<std::endl;
        }
        else if(mesh_algo == "mc")
        {
            const pcl::PCLPointCloud2::ConstPtr input; 
            pcl::PolygonMesh mesh;
            int hoppe_or_rbf=0;
            float iso_level=0.0f;
            int grid_res=0.05;
            float extend_percentage=0.0f;
            float off_surface_displacement=0.0f;

            
            pcl::MarchingCubes<pcl::PointNormal> *mc;
            if (hoppe_or_rbf == 0)
                mc = new pcl::MarchingCubesHoppe<pcl::PointNormal> ();
            else
            {
                mc = new pcl::MarchingCubesRBF<pcl::PointNormal> ();
                (reinterpret_cast<pcl::MarchingCubesRBF<pcl::PointNormal>*> (mc))-> setOffSurfaceDisplacement (off_surface_displacement);
            }

            mc->setIsoLevel (iso_level);
            mc->setGridResolution (grid_res, grid_res, grid_res);
            mc->setPercentageExtendGrid (extend_percentage);
            mc->setInputCloud (pcl_cloud_n);

            mc->reconstruct (mesh);
            delete mc;

            pcl::io::saveOBJFile("output_mesh.obj", mesh);

            // Populate the vertices
            mesh_msg.vertices.resize(mesh.cloud.width * mesh.cloud.height);
            for (size_t i = 0; i < mesh.cloud.width * mesh.cloud.height; ++i) {
                geometry_msgs::Point point;
                point.x = mesh.cloud.data[i * mesh.cloud.point_step + mesh.cloud.fields[0].offset];
                point.y = mesh.cloud.data[i * mesh.cloud.point_step + mesh.cloud.fields[1].offset];
                point.z = mesh.cloud.data[i * mesh.cloud.point_step + mesh.cloud.fields[2].offset];
                mesh_msg.vertices[i] = point;
            }

            // Populate the triangles
            mesh_msg.triangles.resize(mesh.polygons.size());
            for (size_t i = 0; i < mesh.polygons.size(); ++i) {
                shape_msgs::MeshTriangle triangle;
                triangle.vertex_indices[0] = mesh.polygons[i].vertices[0];
                triangle.vertex_indices[1] = mesh.polygons[i].vertices[1];
                triangle.vertex_indices[2] = mesh.polygons[i].vertices[2];
                mesh_msg.triangles[i] = triangle;
            }
            std::cout<<mesh_msg.triangles.size()<<std::endl;

        }
        else if(mesh_algo == "fm")
        {
            pcl::OrganizedFastMesh<pcl::PointXYZ>::Ptr orgMesh (new pcl::OrganizedFastMesh<pcl::PointXYZ>());
            pcl::PolygonMesh mesh;
            // orgMesh.useDepthAsDistance(true);
            orgMesh->setTriangulationType (pcl::OrganizedFastMesh<pcl::PointXYZ>::TRIANGLE_RIGHT_CUT  );
            orgMesh->setInputCloud(pcl_cloud);
            orgMesh->reconstruct(mesh);

            pcl::io::saveOBJFile("output_mesh.obj", mesh);

            // Populate the vertices
            mesh_msg.vertices.resize(mesh.cloud.width * mesh.cloud.height);
            for (size_t i = 0; i < mesh.cloud.width * mesh.cloud.height; ++i) {
                geometry_msgs::Point point;
                point.x = mesh.cloud.data[i * mesh.cloud.point_step + mesh.cloud.fields[0].offset];
                point.y = mesh.cloud.data[i * mesh.cloud.point_step + mesh.cloud.fields[1].offset];
                point.z = mesh.cloud.data[i * mesh.cloud.point_step + mesh.cloud.fields[2].offset];
                mesh_msg.vertices[i] = point;
            }

            // Populate the triangles
            mesh_msg.triangles.resize(mesh.polygons.size());
            for (size_t i = 0; i < mesh.polygons.size(); ++i) {
                shape_msgs::MeshTriangle triangle;
                triangle.vertex_indices[0] = mesh.polygons[i].vertices[0];
                triangle.vertex_indices[1] = mesh.polygons[i].vertices[1];
                triangle.vertex_indices[2] = mesh.polygons[i].vertices[2];
                mesh_msg.triangles[i] = triangle;
            }
            std::cout<<mesh_msg.triangles.size()<<std::endl;
            
        }
        else if(mesh_algo == "ps")
        {
            pcl::Poisson<pcl::PointNormal> poisson;
            pcl::PolygonMesh mesh;

            poisson.setDepth (8);
            poisson.setSolverDivide (8);
            poisson.setIsoDivide (8);
            poisson.setPointWeight (4.0f);
            poisson.setInputCloud (pcl_cloud_n);
            poisson.reconstruct (mesh);

            pcl::io::saveOBJFile("output_mesh.obj", mesh);

            // Populate the vertices
            mesh_msg.vertices.resize(mesh.cloud.width * mesh.cloud.height);
            for (size_t i = 0; i < mesh.cloud.width * mesh.cloud.height; ++i) {
                geometry_msgs::Point point;
                point.x = mesh.cloud.data[i * mesh.cloud.point_step + mesh.cloud.fields[0].offset];
                point.y = mesh.cloud.data[i * mesh.cloud.point_step + mesh.cloud.fields[1].offset];
                point.z = mesh.cloud.data[i * mesh.cloud.point_step + mesh.cloud.fields[2].offset];
                mesh_msg.vertices[i] = point;
            }

            // Populate the triangles
            mesh_msg.triangles.resize(mesh.polygons.size());
            for (size_t i = 0; i < mesh.polygons.size(); ++i) {
                shape_msgs::MeshTriangle triangle;
                triangle.vertex_indices[0] = mesh.polygons[i].vertices[0];
                triangle.vertex_indices[1] = mesh.polygons[i].vertices[1];
                triangle.vertex_indices[2] = mesh.polygons[i].vertices[2];
                mesh_msg.triangles[i] = triangle;
            }
            std::cout<<mesh_msg.triangles.size()<<std::endl;
            
        }
        /*
        else if(mesh_algo == "mc")
        {
            pcl::MarchingCubesHoppe<pcl::PointXYZ> mc;
            mc.setInputCloud(pcl_cloud);

            // Set other parameters as needed
            // mc.setLeafSize(leaf_size, leaf_size, leaf_size);
            // mc.setIsoLevel(iso_level);

            // Create an empty mesh to store the result
            pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);

            // Perform marching cubes surface reconstruction
            mc.reconstruct(*mesh);

            pcl::io::saveOBJFile("output_mesh.obj", *mesh);

            // Populate the vertices
            mesh_msg.vertices.resize(mesh->cloud.width * mesh->cloud.height);
            for (size_t i = 0; i < mesh->cloud.width * mesh->cloud.height; ++i) {
                geometry_msgs::Point point;
                point.x = mesh->cloud.data[i * mesh->cloud.point_step + mesh->cloud.fields[0].offset];
                point.y = mesh->cloud.data[i * mesh->cloud.point_step + mesh->cloud.fields[1].offset];
                point.z = mesh->cloud.data[i * mesh->cloud.point_step + mesh->cloud.fields[2].offset];
                mesh_msg.vertices[i] = point;
            }

            // Populate the triangles
            mesh_msg.triangles.resize(mesh->polygons.size());
            for (size_t i = 0; i < mesh->polygons.size(); ++i) {
                shape_msgs::MeshTriangle triangle;
                triangle.vertex_indices[0] = mesh->polygons[i].vertices[0];
                triangle.vertex_indices[1] = mesh->polygons[i].vertices[1];
                triangle.vertex_indices[2] = mesh->polygons[i].vertices[2];
                mesh_msg.triangles[i] = triangle;
            }
            std::cout<<mesh_msg.triangles.size()<<std::endl;

        }*/

    };

    void publishCentroidMarker(const std::vector<Eigen::Vector4f>& centroids, const std_msgs::Header& header)
    {
        visualization_msgs::Marker marker;
        marker.header = header;
        marker.type = visualization_msgs::Marker::SPHERE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = marker.scale.y = marker.scale.z = 0.1;
        marker.lifetime = ros::Duration(0);
        marker.id = 0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        marker.pose.position.x = 0.0;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        for (const auto& centroid : centroids)
        {
            geometry_msgs::Point point;
            point.x = centroid[0];
            point.y = centroid[1];
            point.z = centroid[2];
            marker.points.push_back(point);
        }

        centroid_marker_pub.publish(marker);
    };

    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*cloud_msg, cloud);

        // Remove collision objects from previos iteration
        if(prev_collisions.size()>0)
        {
            planning_scene_interface.removeCollisionObjects(prev_collisions);
            prev_collisions.clear();
        }

        if (!cloud.empty())
        {
            std::vector<pcl::PointCloud<pcl::PointXYZ>> clusters;
            std::vector<Eigen::Vector4f> centroids;
            createClusters(cloud, clusters, centroids);

            for (size_t i = 0; i < centroids.size(); ++i)
            {
                std::string cluster_name = "cluster_" + std::to_string(i);
                publishCentroidMarker(centroids, cloud_msg->header);
                movementCheck(centroids);
                shape_msgs::Mesh mesh;
                pcl::PointCloud<pcl::PointXYZ>::Ptr clusterPtr;
                clusterPtr = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(clusters[i]);
                createMesh(clusterPtr, mesh);
                addCollisionObject(cluster_name,mesh, cloud_msg->header);
            }
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
    ros::NodeHandle nh;
    ros::Subscriber pcl_sub;
    ros::Publisher safe_stop_pub;
    ros::Publisher centroid_marker_pub;
    ros::Publisher collision_scene_marker_pub;
    pcl::PolygonMesh mesh;
    std::vector<Eigen::Vector4f> prev_centroids;
    int still_count;
    bool still;
    double movement;
    int min_samples;
    double eps;
    int cluster_size_th;
    double voxel_size;
    double alpha;
    int leaf_size;
    std::string mesh_algo;
    std::string algorithm;
    std::string delayluna_method;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    std::vector<std::string> prev_collisions;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "safety_check_2");

    int min_samples = 1;
    double eps = 0.05;
    int cluster_size_th = 50;
    double voxel_size = 0.01;
    double alpha = 0.05;
    int leaf_size = 30;
    std::string mesh_algo = "convex_hull"; // convex_hull (ok), concave_hull, mc, gp3, fm, ps
    SafetyCheck safety_check(min_samples, eps, cluster_size_th, voxel_size, alpha, leaf_size, mesh_algo);

    ros::spin();

    return 0;
};
