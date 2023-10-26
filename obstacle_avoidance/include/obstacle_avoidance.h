#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

#include <sensor_msgs/PointCloud2.h>

#include <std_msgs/Bool.h>
#include <std_msgs/ColorRGBA.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <vector>

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
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/crop_box.h>

#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include <vtkPointData.h>
#include <vtkPolyData.h>
#include <vtkPLYWriter.h>
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkPointData.h>
#include <vtkCellData.h>
#include <vtkDoubleArray.h>
#include <vtkPointLocator.h>
#include <vtkPolyDataNormals.h>
#include <vtkSmoothPolyDataFilter.h>
#include <vtkDelaunay2D.h>
#include <vtkDelaunay3D.h>
#include <vtkDataSetSurfaceFilter.h>
#include <vtkCleanPolyData.h>
#include <vtkSTLWriter.h>
#include <vtkVersion.h>

struct Pose {
    // Constructor to initialize with default values
    static geometry_msgs::Pose ZeroPose() {
        geometry_msgs::Pose pose;
        pose.position.x = 0;
        pose.position.y = 0;
        pose.position.z = 0;
        pose.orientation.x = 0;
        pose.orientation.y = 0;
        pose.orientation.z = 0;
        pose.orientation.w = 1;
        return pose;
    }
    
};

struct Point {
    // Constructor to initialize with default values
    static geometry_msgs::Vector3 VectorXYZ(float x, float y, float z) {
        geometry_msgs::Vector3 point;
        point.x = x;
        point.y = y;
        point.z = z;
        return point;
    }
    
    static geometry_msgs::Point PointXYZ(float x, float y, float z) {
        geometry_msgs::Point point;
        point.x = x;
        point.y = y;
        point.z = z;
        return point;
    }

    static boost::array<uint32_t, 3UL> VertexID(float x, float y, float z) {
        boost::array<uint32_t, 3UL> vertex;
        vertex[0] = x;
        vertex[1] = y;
        vertex[2] = z;
        return vertex;
    }

    
};

struct Color {
    // Constructor to initialize with default values
    static std_msgs::ColorRGBA RGBA(float r, float g, float b, float a) {
        std_msgs::ColorRGBA color;
        color.r = r;
        color.g = g;
        color.b = b;
        color.a = a;
        return color;
    }
    
};

