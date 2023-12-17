#include <ros/ros.h>

#include "obstacle_avoidance.h"
#include "obstacle_avoidance/AvoidanceExecute.h"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include<dmp/GetDMPPlan.h> //GetDMPPlanRequest,LearnDMPFromDemo, LearnDMPFromDemoRequest, SetActiveDMP, SetActiveDMPRequest
#include<dmp/DMPTraj.h>
#include<dmp/DMPData.h>
#include<dmp/DMPPoint.h>

#include<chrono>
#include<thread>
#include <algorithm>


std::vector<std::string> robot_links;
ros::ServiceClient get_dmp_plan;


void get_DMP_plan(trajectory_msgs::JointTrajectory& trajectory,
                      std::vector<double> x_0, 
                      std::vector<double> goal, 
                      double tau=5, 
                      double dt=0.008, 
                      double t_0 = 0, 
                      std::vector<double> initial_velocities = {}, 
                      double seg_length=-1, 
                      int integrate_iter=1, 
                      std::vector<double> goal_thresh = {}){
    
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
        bool plan_received = get_dmp_plan.call(req, res);
        trajectory.joint_names = robot_links;   

        trajectory_msgs::JointTrajectoryPoint jtp;
        for(int i=0; i<res.plan.points.size();i++)
        {
            jtp.positions = res.plan.points[i].positions;
            jtp.velocities = res.plan.points[i].velocities;
            jtp.time_from_start = ros::Duration(res.plan.times[i]);
            trajectory.points.push_back(jtp);
        }
}

void executeCallback(const obstacle_avoidance::AvoidanceExecute::ConstPtr& execute_msg){

    moveit::planning_interface::MoveGroupInterface move_group("manipulator");
    robot_links = move_group.getJointNames();

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

    std::vector<double> joint_group_positions = {0.8067, 0.4054, -0.824, -0.144, 0.0013, 0.4017};

    move_group.setJointValueTarget(joint_group_positions);

    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

    move_group.move();
}    

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_test");
    ros::NodeHandle nh;  
    ros::AsyncSpinner spinner(0);
    spinner.start();

    // Topics
    std::string avoidance_execute_sub_topic;

    // Parameters update
    nh.param<std::string>("avoidance_execute_sub_topic", avoidance_execute_sub_topic, "/avoidance_execute");

    ros::Subscriber execute_subscriber = nh.subscribe(avoidance_execute_sub_topic, 10, executeCallback);

    get_dmp_plan = nh.serviceClient<dmp::GetDMPPlan>("get_dmp_plan");

    ros::waitForShutdown();

    return 0;
}