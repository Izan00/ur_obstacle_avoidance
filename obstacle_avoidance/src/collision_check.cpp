#include <ros/ros.h>
#include "obstacle_avoidance.h"

#include <trajectory_msgs/JointTrajectory.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/GetPlanningScene.h>

#include<chrono>

class CollsionCheck
{
public:
    CollsionCheck(ros::NodeHandle& nh) :nh_(nh),robot_model_loader("robot_description"), robot_state(robot_model_loader.getModel()), planning_scene(robot_model_loader.getModel())
    {
        // Topics
        std::string trajectory_sub_topic;

        // Parameters update
        nh_.param<std::string>("trajectory_sub_topic", trajectory_sub_topic, "/robot_global_trajectory");
        
        // Ros publishers/subscribers
        trajectory_subscriber = nh.subscribe(trajectory_sub_topic, 10, &CollsionCheck::trajectoryCallback, this);

        /*
        robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
        robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
        robot_state::RobotState robot_state(kinematic_model);  

        planning_scene::PlanningScene planning_scene(kinematic_model);
        */
  
        scene_client = nh.serviceClient<moveit_msgs::GetPlanningScene>("get_planning_scene");


    };

    void trajectoryCallback(const trajectory_msgs::JointTrajectory::ConstPtr& trajectory_msg)
    {

        ROS_INFO("Trajectory received");

        auto start_time = std::chrono::high_resolution_clock::now();

        // Create a PlanningScene instance with the current planing scene state
        moveit_msgs::GetPlanningScene::Request req;
        moveit_msgs::GetPlanningScene::Response res;
        if (scene_client.call(req, res))
            planning_scene.setPlanningSceneMsg(res.scene); // apply result to actual PlanningScene

        
        // Create a collision request/response
        collision_detection::CollisionRequest collision_request;
        collision_detection::CollisionResult collision_result;
        collision_request.contacts=true;
        collision_request.distance=true;

        bool collision_found = false;

        // Iterate over the trajectory points
        for(int i=0;i<trajectory_msg->points.size();i++)
        {
            // Create a robot state from the trajectory point
            robot_state.setJointGroupPositions("manipulator", trajectory_msg->points[i].positions);
            //robot_state.printStatePositions();

            // Clear previous collision responses
            collision_result.clear();

            // Request collision and process the result
            planning_scene.checkCollision(collision_request, collision_result, robot_state);
            // planning_scene.checkSelfCollision(collision_request, collision_result, robot_state); 
            if (collision_result.collision) 
            {  
                ROS_ERROR("Collision at path point %d",i);
                robot_state.printStatePositions();
                collision_found = true;
                collision_detection::CollisionResult::ContactMap::const_iterator it;
                for (it = collision_result.contacts.begin(); it != collision_result.contacts.end(); ++it)
                {
                    ROS_INFO("Contact between: %s and %s", it->first.first.c_str(), it->first.second.c_str());
                }
                ROS_INFO("Collision distance: %f", collision_result.distance);
                break;
            } 
        }
        if(collision_found)
        {
            ROS_ERROR("Trajectory is not valid. Collision detected.");
        }
        else
        {
            ROS_INFO("Trajectory is valid.");
        }
        // Stop the timer
        auto end_time = std::chrono::high_resolution_clock::now();

        // Calculate the duration
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

        // Print the duration
        ROS_INFO("Execution time: %lds", duration.count()/1000);


    };


private:
    // Parameters handler
    ros::NodeHandle nh_;

    // Ros publishers/subscribers
    ros::Subscriber trajectory_subscriber;

    robot_model_loader::RobotModelLoader robot_model_loader;
    robot_state::RobotState robot_state;
    planning_scene::PlanningScene planning_scene;

    ros::ServiceClient scene_client;
};



int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_validity_service");
    ros::NodeHandle nh;

    CollsionCheck collsion_check(nh);

    ros::spin();

    return 0;
}
