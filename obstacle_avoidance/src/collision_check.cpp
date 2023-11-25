#include <ros/ros.h>
#include "obstacle_avoidance.h"

#include <trajectory_msgs/JointTrajectory.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit/planning_interface/planning_interface.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>

#include<chrono>
#include <algorithm>

class CollsionCheck
{
public:
    CollsionCheck(ros::NodeHandle& nh) :nh_(nh),
                                        robot_model_loader("robot_description"), 
                                        kinematic_model(robot_model_loader.getModel()), 
                                        robot_state(kinematic_model), planning_scene(kinematic_model), 
                                        control_client("/eff_joint_traj_controller/follow_joint_trajectory", true)
    {
        // Topics
        std::string trajectory_sub_topic;

        // Parameters update
        nh_.param<std::string>("trajectory_sub_topic", trajectory_sub_topic, "/robot_global_trajectory");
        
        nh_.param<int>("collision_check_horizont", collision_check_horizont, 100); // ~10cm (initial_state +  #collision_check_horizont states are checked)
        nh_.param<int>("collision_path_sampling", collision_path_sampling, 10); // ~1cm

        if(collision_check_horizont/collision_path_sampling<1)
            ROS_WARN("Only current/initial state will be collision checked (collision check horizon < collision path path sampling");


        // Ros publishers/subscribers
        trajectory_subscriber = nh.subscribe(trajectory_sub_topic, 10, &CollsionCheck::trajectoryCallback, this);

        /*
        robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
        robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
        robot_state::RobotState robot_state(kinematic_model);  

        planning_scene::PlanningScene planning_scene(kinematic_model);
        */

  
        scene_client = nh.serviceClient<moveit_msgs::GetPlanningScene>("get_planning_scene");

        // Create an action client for FollowJointTrajectoryAction

        // Wait for the action server to start
        ROS_INFO("Waiting for control server to start...");
        control_client.waitForServer();
        ROS_INFO("Control server started");


        };

    bool collision_check(trajectory_msgs::JointTrajectory::ConstPtr trajectory_msg, int start_point=0, int collision_horizon=INT_MAX, int path_sampling=1)
    {
        // Start the timer
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

        // Iterate over the trajectory points
        for(int i=start_point;i<std::min(int(trajectory_msg->points.size()),start_point+collision_horizon+1);i+=path_sampling)
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
                // Stop the timer
                auto end_time = std::chrono::high_resolution_clock::now();
                // Calculate the duration
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
                // Print the duration
                std::cout <<"Execution time: "<<duration.count()/1000.0 << std::endl;

                ROS_ERROR("Collision at path point %d",i);
                robot_state.printStatePositions();
                collision_detection::CollisionResult::ContactMap::const_iterator it;
                for (it = collision_result.contacts.begin(); it != collision_result.contacts.end(); ++it)
                {
                    ROS_WARN("Contact between: %s and %s", it->first.first.c_str(), it->first.second.c_str());
                }
                ROS_WARN("Collision distance: %f", collision_result.distance);

                return true;
            }
        }
        robot_state.setJointGroupPositions("manipulator", trajectory_msg->points[start_point].positions);
     
        //ROS_INFO("No collision detected in path range %d-%d",start_point,start_point+collision_horizon);
        std::cout <<"No collision detected in path range " <<start_point<<"-"<<start_point+collision_horizon<< std::endl;
        
        // Stop the timer
        auto end_time = std::chrono::high_resolution_clock::now();
        // Calculate the duration
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        // Print the duration
        std::cout <<"Execution time: "<<duration.count()/1000.0 << std::endl;

        return false; 

    };

    void trajectoryCallback(const trajectory_msgs::JointTrajectory::ConstPtr& trajectory_msg)
    {
        ROS_INFO("Trajectory received");

        trajectory_msgs::JointTrajectoryPoint target_point = trajectory_msg->points.back();

        //bool results = collision_check(trajectory_msg); //Whole path collsion check

        // Create a goal
        control_msgs::FollowJointTrajectoryGoal goal;
        // Populate the joint names (modify according to your robot's joint names)
        goal.trajectory.joint_names=trajectory_msg->joint_names;

        int trajecotry_size = trajectory_msg->points.size();
        //robot_state.setJointGroupPositions("manipulator", trajectory_msg->points[0].positions);
        for(int i=0;i<trajecotry_size;i++)
        {
            bool collision_result = collision_check(trajectory_msg, i , collision_check_horizont, collision_path_sampling);

            if(collision_result)
            {
                control_client.cancelAllGoals();
                i--;
            }
            else
            {
                // Clear previos trajectory point in goal and add the trajectory point to the goal
                goal.trajectory.points.clear();
                goal.trajectory.points.push_back(trajectory_msg->points[i]);

                // Send the goal
                control_client.sendGoal(goal);

                //std::string state = control_client.getState().toString();
                //ROS_INFO("Goal %d result: %s", i, state.c_str());
            }
        }

        
        // Wait for the action to return
        bool finished_before_timeout = control_client.waitForResult(ros::Duration(10.0));

        if (finished_before_timeout) 
        {
            actionlib::SimpleClientGoalState state = control_client.getState();
            ROS_INFO("Action finished: %s", state.toString().c_str());
        } else 
        {
            ROS_WARN("Action did not finish before the timeout.");
        }
        
        
    };


private:
    // Parameters handler
    ros::NodeHandle nh_;

    // Ros publishers/subscribers
    ros::Subscriber trajectory_subscriber;

    robot_model_loader::RobotModelLoader robot_model_loader;
    robot_model::RobotModelPtr kinematic_model;
    robot_state::RobotState robot_state;
    planning_scene::PlanningScene planning_scene;

    int collision_check_horizont;
    int collision_path_sampling;

    ros::ServiceClient scene_client;
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> control_client;
};



int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_validity_service");
    ros::NodeHandle nh;

    CollsionCheck collsion_check(nh);

    ros::spin();

    return 0;
}
