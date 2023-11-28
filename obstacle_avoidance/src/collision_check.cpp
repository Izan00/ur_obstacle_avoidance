#include <ros/ros.h>
#include "obstacle_avoidance.h"

#include "obstacle_avoidance/AvoidanceExecute.h"

#include <trajectory_msgs/JointTrajectory.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/RobotState.h>

#include<dmp/GetDMPPlan.h> //GetDMPPlanRequest,LearnDMPFromDemo, LearnDMPFromDemoRequest, SetActiveDMP, SetActiveDMPRequest
#include<dmp/DMPTraj.h>
#include<dmp/DMPData.h>
#include<dmp/DMPPoint.h>

#include<chrono>
#include<thread>
#include <algorithm>

class CollsionCheck
{
public:
    CollsionCheck(ros::NodeHandle& nh)
    {
        ROS_INFO("Node starting...");
        // Topics
        std::string trajectory_sub_topic;
        std::string avoidance_execute_sub_topic;

        // Parameters update
        nh.param<std::string>("trajectory_sub_topic", trajectory_sub_topic, "/robot_global_trajectory");
        nh.param<std::string>("avoidance_execute_sub_topic", avoidance_execute_sub_topic, "/avoidance_execute");

        nh.param<double>("collision_threshold", collision_threshold, 0.2); //Set the collisin check horizont distance in m
        
        // Get moveit planning object
        move_group_ptr = new moveit::planning_interface::MoveGroupInterface("manipulator");
        planning_scene_ptr = new planning_scene::PlanningScene(move_group_ptr->getRobotModel());

        // Ros publishers/subscribers
        trajectory_subscriber = nh.subscribe(trajectory_sub_topic, 10, &CollsionCheck::trajectoryCallback, this);
        execute_subscriber = nh.subscribe(avoidance_execute_sub_topic, 10, &CollsionCheck::executeCallback, this);
        
        // Create an action client for DMP
        get_dmp_plan_client = nh.serviceClient<dmp::GetDMPPlan>("get_dmp_plan");

        // Create an action client for planning scene
        scene_client = nh.serviceClient<moveit_msgs::GetPlanningScene>("get_planning_scene");

        ROS_INFO("Node started");

    };

    double getCollisionDistance(const std::vector<trajectory_msgs::JointTrajectoryPoint>& trajectory_points, int path_start=0) 
    {   
        moveit_msgs::GetPlanningScene::Request req;
        moveit_msgs::GetPlanningScene::Response res;
        if (scene_client.call(req, res))
            planning_scene_ptr->setPlanningSceneMsg(res.scene);

        collision_detection::CollisionRequest collision_request;
        collision_detection::CollisionResult collision_result;

        collision_detection::CollisionResult::ContactMap contacts;

        // Start the timer
        auto start_time = std::chrono::high_resolution_clock::now();
        // Check each point in the trajectory for collision
        robot_state::RobotState robot_state = *move_group_ptr->getCurrentState();

        double collision_distance = -1;

        for (int i=path_start;i<trajectory_points.size();i++)
        {
            //std::cout<<"Point "<<i<<" checked"<<std::endl;
            // Set the robot state to the current trajectory point
            robot_state.setJointGroupPositions(move_group_ptr->getRobotModel()->getJointModelGroup("manipulator"),
                                            trajectory_points[i].positions);

            // Update the Planning Scene with the current robot state
            planning_scene_ptr->setCurrentState(robot_state);

            //double distance = current_state->distance(robot_state);
            // Collision checking
            // Clear previous collision responses
            collision_result.clear();
            planning_scene_ptr->checkCollision(collision_request, collision_result);

            
            // If collision occurs, the path is not valid
            if (collision_result.collision) {
                
                planning_scene_ptr->getCollidingPairs(contacts);
                
                for (const auto& contact : contacts) {
                    const std::string& object1 = contact.first.first;
                    const std::string& object2 = contact.first.second;
                    //std::cout<<"Collision between: " << object1 << " and " << object2<<std::endl;

                    // Print collision points
                    for (const auto& contact_point : contact.second) {
                        const Eigen::Vector3d& point = contact_point.pos;
                        //std::cout<<"Collision point (collison state, base_link frame): " << point.x() << ", " << point.y() << ", " << point.z()<<std::endl;

                        // Get the global transform of a reference link in the source state
                        Eigen::Affine3d source_transform = robot_state.getGlobalLinkTransform(object1);

                        // Transform the point from the source state to the reference link frame
                        Eigen::Vector3d transformed_point = source_transform.inverse() * point;
                        //std::cout<<"Collision point (collison state, link frame): " << transformed_point.x() << ", " << transformed_point.y() << ", " << transformed_point.z()<<std::endl;

                        // Get the global transform of the reference link in the target state
                        robot_state = *move_group_ptr->getCurrentState();
                        Eigen::Affine3d target_transform = robot_state.getGlobalLinkTransform(object1);

                        // Transform the point from the reference link frame to the target state
                        transformed_point = target_transform * transformed_point;
                        //std::cout<<"Collision point (curent state, base_link frame): " << transformed_point.x() << ", " << transformed_point.y() << ", " << transformed_point.z()<<std::endl;

                        collision_distance = (transformed_point - point).norm();
                        //std::cout<<"Collsision_distance: "<< collision_distance<<std::endl;
                    }
                }

                // Stop the timer
                auto end_time = std::chrono::high_resolution_clock::now();
                // Calculate the duration
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
                // Print the duration
                //std::cout <<"Execution time: "<<duration.count()/1000.0 << std::endl;
                
                return collision_distance;
                //return false;
            }
        }
        // Stop the timer
        auto end_time = std::chrono::high_resolution_clock::now();
        // Calculate the duration
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        // Print the duration
        //std::cout <<"Execution time: "<<duration.count()/1000.0 << std::endl;

        // If no collisions occurred in any trajectory point, the path is valid
        return collision_distance;
        //return true;
    };
    
    moveit::planning_interface::MoveGroupInterface::Plan getPlanFromTrajectory(const trajectory_msgs::JointTrajectory::ConstPtr trajectory)
    {
        moveit_msgs::RobotState robot_state_msg;
        moveit::core::robotStateToRobotStateMsg(*move_group_ptr->getCurrentState(), robot_state_msg);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_.joint_trajectory = *trajectory;
        plan.start_state_  = robot_state_msg;
        return plan;    
    };

    void executeCallback(const obstacle_avoidance::AvoidanceExecute::ConstPtr& execute_msg){
        //Generate a plan...            
        //Used: initial_pose, goal_pose, seg_length=-1, initial_velocities=[], t_0 = None, tau=5, dt=0.008, integrate_iter=1,goal_thresh=[]):               
        std::vector<double> initial_pose =  execute_msg->start_point.positions;
        double tau =  execute_msg->tau;
        double dt =  execute_msg->dt;
        std::vector<double> goal_pose =  execute_msg->target_point.positions;

        trajectory_msgs::JointTrajectory trajectory;

        getDmpPlan(trajectory, initial_pose,goal_pose,tau,dt);
        
        trajectory_msgs::JointTrajectory::ConstPtr trajectory_ptr =  boost::make_shared<trajectory_msgs::JointTrajectory>(trajectory);
        
        std::cout<<"Starting plan..."<<std::endl;
        double collision_distance = -1;

        move_group_ptr->asyncExecute(getPlanFromTrajectory(trajectory_ptr));

        int current_point = 0;
        while(current_point<trajectory.points.size()-1)
        {
            current_point = getCurrentTrajectoryPoint(trajectory.points);
            collision_distance = getCollisionDistance(trajectory.points, current_point);            
            std::cout<<"Current point: "<<current_point<<" - Collision dist: "<<collision_distance<<std::endl;
            if(collision_distance>0 && collision_distance<collision_threshold)
            {
                std::cout<<"Stoped"<<std::endl;
                move_group_ptr->stop(); // has some delay btween sent and stoped
                while(collision_distance>0 && collision_distance<collision_threshold)
                {
                    current_point = getCurrentTrajectoryPoint(trajectory.points);
                    collision_distance = getCollisionDistance(trajectory.points, current_point);
                    std::cout<<"Current point: "<<current_point<<" - Collision dist: "<<collision_distance<<std::endl;
                }
                move_group_ptr->asyncExecute(getPlanFromTrajectory(trajectory_ptr));
            }

        }
    };


    int getCurrentTrajectoryPoint(const std::vector<trajectory_msgs::JointTrajectoryPoint> trajectory_points) 
    {
        // Initialize variables to store the closest point and its distance
        double min_distance = std::numeric_limits<double>::max();
        int closest_point = -1;
        robot_state::RobotState robot_state = *move_group_ptr->getCurrentState();
        // Iterate through each point in the trajectory
        for (int i=0;i<trajectory_points.size();i++) {
            // Convert the trajectory point to RobotState
            robot_state.setJointGroupPositions(move_group_ptr->getRobotModel()->getJointModelGroup("manipulator"),
                                                trajectory_points[i].positions);

            // Calculate the distance between the current state and the trajectory point
            double distance = move_group_ptr->getCurrentState()->distance(robot_state);

            // Update the closest point if this point is closer
            if (distance < min_distance) {
                min_distance = distance;
                closest_point = i;
            }
        }

        return closest_point;
    };


    void getDmpPlan(trajectory_msgs::JointTrajectory& trajectory, std::vector<double> x_0, 
                      std::vector<double> goal, double tau=5, double dt=0.008, double t_0 = 0,  
                      std::vector<double> initial_velocities = {}, double seg_length=-1, 
                      int integrate_iter=1, std::vector<double> goal_thresh = {})
    {
    
        std::vector<double> x_dot_0(x_0.size(), 0.0);
        tau = tau*2;
        
        if(goal_thresh.size()==0)
            goal_thresh.resize(x_0.size(), 0.01);
    
        dmp::GetDMPPlan::Request req;
        req.x_0=x_0; 
        req.x_dot_0=x_dot_0;
        req.t_0=t_0;
        req.goal=goal;
        req.goal_thresh=goal_thresh;
		req.seg_length=seg_length;
        req.tau=tau;
        req.dt=dt;
        req.integrate_iter=integrate_iter;

		dmp::GetDMPPlan::Response res; 
        bool plan_received = get_dmp_plan_client.call(req, res);
        trajectory.joint_names = move_group_ptr->getRobotModel()->getJointModelGroup("manipulator")->getVariableNames();   

        trajectory_msgs::JointTrajectoryPoint jtp;
        for(int i=0; i<res.plan.points.size();i++)
        {
            jtp.positions = res.plan.points[i].positions;
            jtp.velocities = res.plan.points[i].velocities;
            jtp.time_from_start = ros::Duration(res.plan.times[i]);
            trajectory.points.push_back(jtp);
        }
    };


    void trajectoryCallback(const trajectory_msgs::JointTrajectory::ConstPtr& trajectory_msg)
    {
        ROS_INFO("Trajectory received");

        trajectory_msgs::JointTrajectoryPoint target_point = trajectory_msg->points.back();

        //bool results = collision_check(trajectory_msg); //Whole path collsion check
        
    };


private:
    // Ros publishers/subscribers
    ros::Subscriber trajectory_subscriber;
    ros::Subscriber execute_subscriber;
   
    planning_scene::PlanningScene *planning_scene_ptr;
    moveit::planning_interface::MoveGroupInterface *move_group_ptr; 

    double collision_threshold;

    ros::ServiceClient get_dmp_plan_client;
    ros::ServiceClient scene_client;
    
};



int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_validity_service");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(0); // 0 means max number of threads available
    spinner.start();  // start the AsyncSpinner asynchronously (non-blocking)

    CollsionCheck collsion_check(nh);

    //ros::spin();
    ros::waitForShutdown();
  

    return 0;
}