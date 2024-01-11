#include <ros/ros.h>
#include "obstacle_avoidance.h"

#include <string>
#include <iostream>


#include<dmp/GetDMPPlan.h> 
#include<dmp/LearnDMPFromDemo.h> 
#include<dmp/SetActiveDMP.h> 
#include<dmp/DMPTraj.h>
#include<dmp/DMPData.h>
#include<dmp/DMPPoint.h>

class CollsionCheck
{
public:
    CollsionCheck(ros::NodeHandle& nh)
    {
        ROS_INFO("Node starting...");
        // Topics
        std::string trajectory_sub_topic;
        std::string avoidance_execute_sub_topic;
        std::string robot_execution_status_topic;

        std::string cartesian_dmp_service_namespace;

        double cartesain_dmp_gamma_obs; 
        double cartesain_dmp_beta_obs; 
        double cartesain_dmp_k_obs; 
        double cartesain_dmp_gamma_np; 
        double cartesain_dmp_beta_np; 
        double cartesain_dmp_k_np; 
        double cartesain_dmp_gamma_d; 
        double cartesain_dmp_k_d; 

        // Parameters update
        nh.param<std::string>("trajectory_sub_topic", trajectory_sub_topic, "/robot_global_trajectory");
        nh.param<std::string>("avoidance_execute_sub_topic", avoidance_execute_sub_topic, "/avoidance_execute");
        nh.param<std::string>("robot_execution_status_topic", robot_execution_status_topic, "/robot_execution_status");
        
        nh.param<bool>("debug", debug, false);

        nh.param<double>("collision_threshold", collision_threshold, 0.25); //Set the collisin check horizont distance in m
        nh.param<double>("goal_threshold", goal_threshold, 0.03); // Joints L2 norm in m
        nh.param<double>("collision_check_sampling", collision_check_sampling, 0.02);


        nh.param<double>("cartesian_dmp_speed_scale", cartesian_dmp_speed_scale, 1.0); 
        nh.param<double>("cartesian_dmp_dt", cartesian_dmp_dt, 0.01); 

        nh.param<double>("cartesain_dmp_K", cartesain_dmp_K, 100); 
        nh.param<double>("cartesain_dmp_D", cartesain_dmp_D, 2.0*pow(cartesain_dmp_K,0.5)); 

        nh.param<double>("cartesain_dmp_scale_m", cartesain_dmp_scale_m, 0.0); 
        nh.param<double>("cartesain_dmp_scale_n", cartesain_dmp_scale_n, 1.0); 

        nh.param<double>("cartesain_dmp_gamma_obs", cartesain_dmp_gamma_obs, 200.0); 
        nh.param<double>("cartesain_dmp_beta_obs", cartesain_dmp_beta_obs, 5.0); 
        nh.param<double>("cartesain_dmp_k_obs", cartesain_dmp_k_obs, 6.0); 
        nh.param<double>("cartesain_dmp_gamma_np", cartesain_dmp_gamma_np, 500.0); 
        nh.param<double>("cartesain_dmp_beta_np", cartesain_dmp_beta_np, 4.0); 
        nh.param<double>("cartesain_dmp_k_np", cartesain_dmp_k_np, 5.0); 
        nh.param<double>("cartesain_dmp_gamma_d", cartesain_dmp_gamma_d, 15.0); 
        nh.param<double>("cartesain_dmp_k_d", cartesain_dmp_k_d, 8.0); 

        cartesain_dmp_gamma.push_back(cartesain_dmp_gamma_obs);
        cartesain_dmp_gamma.push_back(cartesain_dmp_gamma_np);
        cartesain_dmp_gamma.push_back(cartesain_dmp_gamma_d);
        cartesain_dmp_beta.push_back(cartesain_dmp_beta_obs);
        cartesain_dmp_beta.push_back(cartesain_dmp_beta_np);
        cartesain_dmp_k.push_back(cartesain_dmp_k_obs);
        cartesain_dmp_k.push_back(cartesain_dmp_k_np);
        cartesain_dmp_k.push_back(cartesain_dmp_k_d);

        nh.param<int>("cartesian_dmp_n_bases", cartesian_dmp_n_bases, 50);  
        nh.param<double>("ik_jump_threshold_factor", ik_jump_threshold_factor, 0.05);

        nh.param<std::string>("cartesian_dmp_service_namespace", cartesian_dmp_service_namespace, "cartesian"); 
        
        nh.param<std::string>("end_effector_link", end_effector_link, "rg2_eef_link");
        nh.param<std::string>("end_effector_collision_link", end_effector_collision_link, "rg2_body_link");
        
        // Get moveit planning object
        move_group_ptr = new moveit::planning_interface::MoveGroupInterface("manipulator");
        planning_scene_ptr = new planning_scene::PlanningScene(move_group_ptr->getRobotModel());

        move_group_ptr->setEndEffectorLink(end_effector_link);

        // Ros publishers/subscribers
        dmp_trajectory_subscriber = nh.subscribe(trajectory_sub_topic, 10, &CollsionCheck::trajectoryCallback, this);
        
        robot_execution_status_publisher = nh.advertise<std_msgs::Int32>(robot_execution_status_topic, 10);
        cartesian_dmp_path_publisher = nh.advertise<nav_msgs::Path>("/imitated_path_avoidance",10);
        cartesian_ik_path_publisher = nh.advertise<nav_msgs::Path>("/true_imitated_path_avoidance",10);

        // Create action clients for DMP
        if(cartesian_dmp_service_namespace.size()>0)
            cartesian_dmp_service_namespace = cartesian_dmp_service_namespace+"/";

        std::cout<<cartesian_dmp_service_namespace+"get_dmp_plan"<<std::endl;
        get_dmp_plan_client = nh.serviceClient<dmp::GetDMPPlan>(cartesian_dmp_service_namespace+"get_dmp_plan");
        learn_dmp_from_demo_client = nh.serviceClient<dmp::LearnDMPFromDemo>(cartesian_dmp_service_namespace+"learn_dmp_from_demo");
        set_active_dmp_client = nh.serviceClient<dmp::SetActiveDMP>(cartesian_dmp_service_namespace+"set_active_dmp");

        // Create an action client for planning scene
        scene_client = nh.serviceClient<moveit_msgs::GetPlanningScene>("get_planning_scene");

        ROS_INFO("Node started");

    };

    double getCollisionDistance(const std::vector<trajectory_msgs::JointTrajectoryPoint>& trajectory_points, std::string& collision_object, std::string& collision_link, std::vector<double>& collision_point, int path_start=0)
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
        Eigen::Vector3d e_collision_point;

        Eigen::Vector3d prev_end_effector_position;

        double distance = 0;
        int total_checked = 0;
        for (int i=path_start;i<trajectory_points.size();i++)
        {
            // Set the robot state to the current trajectory point
            robot_state.setJointGroupPositions(move_group_ptr->getRobotModel()->getJointModelGroup("manipulator"),
                                            trajectory_points[i].positions);

            // Update the Planning Scene with the current robot state
            planning_scene_ptr->setCurrentState(robot_state);

            if (i==path_start){
                const Eigen::Isometry3d& initial_end_effector_pose = robot_state.getGlobalLinkTransform(end_effector_link);
                prev_end_effector_position = initial_end_effector_pose.translation();
            }
            const Eigen::Isometry3d& end_effector_pose = robot_state.getGlobalLinkTransform(end_effector_link);
            distance = (end_effector_pose.translation() - prev_end_effector_position).norm();
            //std::cout<<"Check dist: " <<distance<<std::endl;
            if (distance > collision_check_sampling){
                total_checked++;
                //std::cout<<"Point: " <<i <<" Checked: "<<total_checked<<"/"<<trajectory_points.size()-path_start<<std::endl;
                prev_end_effector_position = end_effector_pose.translation();
                // Collision checking
                // Clear previous collision responses
                collision_result.clear();
                planning_scene_ptr->checkCollision(collision_request, collision_result);

                // If collision occurs
                if (collision_result.collision) 
                {
                    //std::cout<<i<<std::endl;
                    planning_scene_ptr->getCollidingPairs(contacts);
                    for (const auto& contact : contacts) {
                        const std::string& object1 = contact.first.first;
                        const std::string& object2 = contact.first.second;
                        //std::cout<<"Collision between: " << object1 << " and " << object2<<std::endl;
                        if(object1.find("collision_cluster_") != std::string::npos || object1.find("collision_cluster_") != std::string::npos)
                        //if(object1==end_effector_collision_link || object2==end_effector_collision_link)
                        {
                            // Print collision points
                            for (const auto& contact_point : contact.second) {
                                const Eigen::Vector3d& point = contact_point.pos;
                                //std::cout<<"Collision point (collison state, base_link frame): " << point.x() << ", " << point.y() << ", " << point.z()<<std::endl;

                                // Get the global transform of a reference link in the source state
                                Eigen::Affine3d source_transform = robot_state.getGlobalLinkTransform(end_effector_collision_link);

                                // Transform the point from the source state to the reference link frame
                                Eigen::Vector3d transformed_point = source_transform.inverse() * point;
                                //std::cout<<"Collision point (collison state, link frame): " << transformed_point.x() << ", " << transformed_point.y() << ", " << transformed_point.z()<<std::endl;

                                // Get the global transform of the reference link in the target state
                                robot_state = *move_group_ptr->getCurrentState();
                                Eigen::Affine3d target_transform = robot_state.getGlobalLinkTransform(end_effector_collision_link);

                                // Transform the point from the reference link frame to the target state
                                transformed_point = target_transform * transformed_point;
                                //std::cout<<"Collision point (curent state, base_link frame): " << transformed_point.x() << ", " << transformed_point.y() << ", " << transformed_point.z()<<std::endl;
                                double new_distance = (transformed_point - point).norm();
                                if(collision_distance>new_distance || collision_distance<0)
                                {   
                                    collision_distance = new_distance;
                                    e_collision_point = point;
                                }
                            }
                            //std::cout<<"Collision_distance: "<< collision_distance<<std::endl;
                            // Stop the timer
                            auto end_time = std::chrono::high_resolution_clock::now();
                            // Calculate the duration
                            double duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count()/1000.0;
                            // Print the duration
                            //std::cout <<"Execution time: "<<duration << std::endl;
                            
                            if(object1.find("collision_cluster_") != std::string::npos)
                            {
                                collision_object = object1;
                                collision_link = object2;
                            }
                            else
                            {
                                collision_object = object2;
                                collision_link = object1;
                            }
                            collision_point = std::vector<double>{e_collision_point.x(),e_collision_point.y(),e_collision_point.z()};
                            return collision_distance;
                            //return false;
                        }
                    }
                }
            }
        }
        //std::cout<<"Checked: "<<total_checked<<"/"<<trajectory_points.size()-path_start<<std::endl;
                

         // Stop the timer
        auto end_time = std::chrono::high_resolution_clock::now();
        // Calculate the duration
        double duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count()/1000.0;
        // Print the duration
        //std::cout <<"Collision check execution time: "<<duration<< std::endl;

        // If no collisions occurred in any trajectory point, the path is valid
        return collision_distance;
        //return true;
    };

    moveit::planning_interface::MoveGroupInterface::Plan getPlanFromJointTrajectory(const trajectory_msgs::JointTrajectory::ConstPtr trajectory, int initial_point=0)
    {
        moveit_msgs::RobotState robot_state_msg;
        moveit::core::robotStateToRobotStateMsg(*move_group_ptr->getCurrentState(), robot_state_msg);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        
        if (initial_point>0)
        {
            std::vector<trajectory_msgs::JointTrajectoryPoint>  points = trajectory->points;
            std::vector<trajectory_msgs::JointTrajectoryPoint> trimed_points(points.begin() + initial_point, points.end());
            plan.trajectory_.joint_trajectory.points = trimed_points;
        
        }
        else
            plan.trajectory_.joint_trajectory.points = trajectory->points;

        plan.trajectory_.joint_trajectory.joint_names = trajectory->joint_names;
        plan.start_state_  = robot_state_msg;
        trajectory_msgs::JointTrajectoryPoint start_point;
        start_point.positions = robot_state_msg.joint_state.position;
        start_point.velocities = std::vector<double>(6,0);
        start_point.time_from_start = ros::Duration(0.0);
        plan.trajectory_.joint_trajectory.points.insert(plan.trajectory_.joint_trajectory.points.begin(), start_point);

        return plan;    
    };

    double computePoseMsgDistance(const geometry_msgs::Pose& pose1, const geometry_msgs::Pose& pose2) {
        // Extract x, y, and z coordinates from each Pose
        double x1 = pose1.position.x;
        double y1 = pose1.position.y;
        double z1 = pose1.position.z;

        double x2 = pose2.position.x;
        double y2 = pose2.position.y;
        double z2 = pose2.position.z;

        // Compute Euclidean distance
        double distance = std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2) + std::pow(z2 - z1, 2));

        return distance;
    };

    double computePoseVectorDistance(const std::vector<double>& pose1, const std::vector<double>& pose2) {
     
        // Compute Euclidean distance
        double distance = std::sqrt(std::pow(pose2[0] - pose1[0], 2) + std::pow(pose2[1] - pose1[1], 2) + std::pow(pose2[2] - pose1[2], 2));

        return distance;
    };

    trajectory_msgs::JointTrajectory cartesianPathToJointTrajectory(std::vector<geometry_msgs::Pose>& positions, double dt, double start_dist_th=0, double speed_scaling=1.0)
    {
        std::cout<<"Positions size: "<<positions.size()<<std::endl;
        
        /* Interface converter (not working)
        moveit_msgs::RobotTrajectory robot_trajectory;
        moveit_msgs::MoveItErrorCodes *error;
        std::vector<geometry_msgs::Pose> spaced_pose;
        spaced_pose.push_back(positions[0]);
        for(std::size_t i = 1; i <= positions.size(); ++i)
        {
            std::cout<<"Dist: "<<computePoseMsgDistance(positions[i],spaced_pose.back())<<std::endl;
            if(computePoseMsgDistance(positions[i],spaced_pose.back())>0.01)
                spaced_pose.push_back(positions[i]);
        }
        std::cout<<"Spaced size: "<<spaced_pose.size()<<std::endl;
        */

        //double fraction = move_group_ptr->computeCartesianPath(positions, 0.01, 0.0, robot_trajectory, true, error);
        //std::cout<<"Plan fraction: "<<fraction<<std::endl;

        robot_state::RobotState start_state = *move_group_ptr->getCurrentState();

        std::vector<moveit::core::RobotStatePtr> traj;
        const robot_model::JointModelGroup* group = start_state.getJointModelGroup("manipulator");
        const robot_model::LinkModel* link = start_state.getLinkModel(end_effector_link);

        // Cartesian pose we start from
        Eigen::Isometry3d start_pose = start_state.getGlobalLinkTransform(link);
        
        const std::vector<const moveit::core::JointModel*>& cjnt = group->getContinuousJointModels();
        // make sure that continuous joints wrap
        for (const moveit::core::JointModel* joint : cjnt)
            start_state.enforceBounds(joint);

        moveit::core::JumpThreshold jump_threshold = moveit::core::JumpThreshold(ik_jump_threshold_factor);

        robot_state::GroupStateValidityCallbackFn constraint_fn;
        kinematics::KinematicsQueryOptions options;

        // To limit absolute joint-space jumps, we pass consistency limits to the IK solver
        std::vector<double> consistency_limits;
        if (jump_threshold.prismatic > 0 || jump_threshold.revolute > 0)
        {
            for (const moveit::core::JointModel* jm : group->getActiveJointModels())
            {
            double limit;
            switch (jm->getType())
            {
                case moveit::core::JointModel::REVOLUTE:
                limit = jump_threshold.revolute;
                break;
                case moveit::core::JointModel::PRISMATIC:
                limit = jump_threshold.prismatic;
                break;
                default:
                limit = 0.0;
            }
            if (limit == 0.0)
                limit = jm->getMaximumExtent();
            consistency_limits.push_back(limit);
            }
        }

        traj.clear();
        traj.push_back(std::make_shared<moveit::core::RobotState>(start_state));

        int steps = positions.size();
        std::cout<<"Computing path IK..."<<std::endl;
        double dist_prev_point=0;
        bool success=false;
        int last_valid_ik = 0;
        robot_trajectory::RobotTrajectory rt(start_state.getRobotModel(), "manipulator");

        for(std::size_t i = 0; i <= steps; ++i)
        {
            Eigen::Isometry3d eigen_pose;
            tf::poseMsgToEigen(positions[i], eigen_pose);
            double distance = (eigen_pose.translation() - start_pose.translation()).norm();
            //std::cout<<i<<"/"<<steps<<" - Dist: "<<distance<<std::endl;
            if(distance>start_dist_th)
                success = start_state.setFromIK(group, eigen_pose, link->getName(), consistency_limits, 0.0, constraint_fn, options);
            
            if(success)
            {
                if(traj.size()>0)
                    dist_prev_point = std::make_shared<moveit::core::RobotState>(start_state)->distance(*traj.back(), group);
                //if(dist_prev_point<0.02)
                traj.push_back(std::make_shared<moveit::core::RobotState>(start_state));
                //std::cout<<"Dist: "<<dist_prev_point<<std::endl;
                double current_dt = (i-last_valid_ik)*dt;
                // Avoid abrupt accelerations at start
                if(current_dt==0) // If the first point is taken, it will be scaled 3 time
                    current_dt=dt/speed_scaling;
				if(distance<start_dist_th*3) // During a distance of 3 times the start distance threshold, it will be scaled 2 times 
					current_dt=current_dt/speed_scaling;
                // Add time to trajectory (custom)
                rt.addSuffixWayPoint(traj.back(), current_dt/speed_scaling);
                last_valid_ik=i;
            }
            //std::cout<<i<<"/"<<steps<<" - Traj size: "<<traj.size()<<std::endl;
        }
        std::cout<<"IK path size: "<<traj.size()<<std::endl;
		
        
        // Jumps check (not working)
        std::vector<moveit::core::RobotStatePtr> traj_bounded = traj;
        moveit::core::CartesianInterpolator::checkJointSpaceJump(group, traj_bounded, jump_threshold);
        std::cout<<"Bounded IK path size (not used): "<<traj_bounded.size()<<std::endl;
        
        /*
        // Add time to trajectory (auto)
        trajectory_processing::IterativeParabolicTimeParameterization time_param;
        if(method=="pose" || method=="eigen")
            time_param.computeTimeStamps(rt, cartesian_dmp_speed_scale,cartesian_dmp_speed_scale); // scale speed for sim
        */

        // Extract trajectrory
        moveit_msgs::RobotTrajectory solution;
        rt.getRobotTrajectoryMsg(solution);
        trajectory_msgs::JointTrajectory trajectory = solution.joint_trajectory;

        // Check trajectory IK conversion
        robot_state::RobotState robot_state = *move_group_ptr->getCurrentState();
        trajectory_msgs::JointTrajectoryPoint trajectory_point;
        std::vector<geometry_msgs::PoseStamped> stamped_positions;
        geometry_msgs::PoseStamped stamped_pose;
        Eigen::Vector3d prev_end_effector_pose;
        double distance = 0;
        double speed = 0;
        for (int i=0;i<trajectory.points.size(); i++)
        {
            trajectory_point = trajectory.points[i];
            // Set robot state based on joint values
            robot_state.setJointGroupPositions(move_group_ptr->getRobotModel()->getJointModelGroup("manipulator"),
                                            trajectory_point.positions);
            // Get end effector pose
            const Eigen::Isometry3d& end_effector_pose = robot_state.getGlobalLinkTransform(end_effector_link);
            Eigen::Vector3d end_effector_position = end_effector_pose.translation();
            Eigen::Quaterniond end_effector_orientation(end_effector_pose.rotation());
            geometry_msgs::Pose pose;
            pose.position.x=end_effector_position.x();
            pose.position.y=end_effector_position.y();
            pose.position.z=end_effector_position.z();
            stamped_pose.pose=pose; // Ignore eef orientation to avoid path visualization error in Rviz

            if(i>0)
            {
                distance = (end_effector_pose.translation() - prev_end_effector_pose).norm();
                speed = distance/(trajectory.points[i].time_from_start.toNSec()-trajectory.points[i-1].time_from_start.toNSec())*1e9;
                if(debug)
                    std::cout<<"Point "<<i<<" distance: "<<distance<<" speed: "<<speed<<std::endl;
            }
            if(speed>0.1)
                trajectory.points.erase (trajectory.points.begin() + i);
            else
            {
                stamped_positions.push_back(stamped_pose);
                prev_end_effector_pose = end_effector_pose.translation();
            }

        }
        nav_msgs::Path imitated_path;
        imitated_path.header.frame_id = "base_link";
        imitated_path.poses = stamped_positions;
        cartesian_ik_path_publisher.publish(imitated_path);
        
        return trajectory;
    };

    void executePlanWithAvoidance(trajectory_msgs::JointTrajectory::Ptr trajectory_ptr)
    {
        std::cout<<"Starting plan..."<<std::endl;
  
        std_msgs::Int32 robot_exectuion_status; //0-None 1-stopped 2-running 3-success 4-failed
        robot_exectuion_status.data = 2;
        robot_execution_status_publisher.publish(robot_exectuion_status);

        double execution_time = trajectory_ptr->points.back().time_from_start.toSec() - trajectory_ptr->points.front().time_from_start.toSec();
        std::cout<<"Execution time: "<<execution_time<<std::endl;
        double collision_distance = -1;
        std::string collision_object_name = "";
        std::string collision_link_name = "";
        
        // Get goal state
        robot_state::RobotState goal_robot_state = *move_group_ptr->getCurrentState();
        goal_robot_state.setJointGroupPositions(move_group_ptr->getRobotModel()->getJointModelGroup("manipulator"),
                                            trajectory_ptr->points.back().positions);
		const Eigen::Isometry3d& goal_effector_pose = goal_robot_state.getGlobalLinkTransform(end_effector_link);
        Eigen::Vector3d goal_effector_position = goal_effector_pose.translation();
        robot_state::RobotState current_state = *move_group_ptr->getCurrentState();
        const Eigen::Isometry3d& current_effector_pose = current_state.getGlobalLinkTransform(end_effector_link);
        double goal_dist = (goal_effector_position - current_effector_pose.translation()).norm();
        //double goal_dist = current_state.distance(goal_robot_state);
            
        // Start movement
        //move_group_ptr->asyncExecute(getPlanFromJointTrajectory(trajectory_ptr,0));
        moveit_msgs::MoveItErrorCodes error = move_group_ptr->asyncExecute(getPlanFromJointTrajectory(trajectory_ptr,0));
        //moveit_msgs::MoveItErrorCodes error = move_group_ptr->execute(getPlanFromJointTrajectory(trajectory_ptr,0));
        
        if(error.val!=1)
            std::cout<<"Error: "<<error.val<<std::endl;
        std::cout<<"Moving..."<<std::endl;
        auto execution_start_time = std::chrono::high_resolution_clock::now();
        int current_point = 0;
        std::vector<double> collision_point;
        std::cout<<"Start time"<<trajectory_ptr->points[0].time_from_start<<std::endl;
        auto st = std::chrono::high_resolution_clock::now();
        int collision_count=0;
        while(current_point<trajectory_ptr->points.size()-1 && goal_dist>goal_threshold)
        {  
            robot_state::RobotState current_state = *move_group_ptr->getCurrentState();
            const Eigen::Isometry3d& current_effector_pose = current_state.getGlobalLinkTransform(end_effector_link);
			goal_dist = (goal_effector_position - current_effector_pose.translation()).norm();
			
            // Calculate the duration
            double execution_duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - execution_start_time).count()/1000.0;
            if(execution_duration> execution_time+10) // 5s threshold
            {   
                std::cout<<"Execution error"<<std::endl;
                robot_exectuion_status.data = 4;
                robot_execution_status_publisher.publish(robot_exectuion_status);
                move_group_ptr->stop();
                return;
            }
            //auto start_time = std::chrono::high_resolution_clock::now();
            current_point = getCurrentTrajectoryPoint(trajectory_ptr, current_point);
            //auto end_time = std::chrono::high_resolution_clock::now();
            //double duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count()/1000.0;
            //std::cout<<"Trajectory loc execution time: "<<duration<< std::endl;

            collision_distance = getCollisionDistance(trajectory_ptr->points, collision_object_name, collision_link_name, collision_point, current_point);        
           
            //std::cout<<"Current point: "<<current_point<<"/"<<trajectory_ptr->points.size()<<" - Collision dist: "<<collision_distance<<std::endl;
            if(debug)
                std::cout<<"Current point: "<<current_point<<"/"<<trajectory_ptr->points.size()<<" - Collision dist: "<<collision_distance<<" - Duration: "<<execution_duration<<" -  Goal dist: "<<goal_dist<<std::endl;
            if(collision_distance>0 && collision_distance<collision_threshold)
            {   
				collision_count++;
				if (collision_count>=5){
					collision_count=0;
					std::cout<<"Current point: "<<current_point<<"/"<<trajectory_ptr->points.size()<<" - Collision dist: "<<collision_distance<<" - Duration: "<<execution_duration<<" -  Goal dist: "<<goal_dist<<std::endl;
					std::cout<<"Stoped"<<std::endl;
					move_group_ptr->stop(); // has some delay btween sent and stoped
					auto stop_start_time = std::chrono::high_resolution_clock::now();
					robot_exectuion_status.data = 1;
					robot_execution_status_publisher.publish(robot_exectuion_status);
					sleep(5); // Compensate stop delay
					current_point = getCurrentTrajectoryPoint(trajectory_ptr,current_point);
					collision_distance = getCollisionDistance(trajectory_ptr->points, collision_object_name, collision_link_name,collision_point,current_point);            
                    //std::cout<<"Current point: "<<current_point<<"/"<<trajectory_ptr->points.size()<<" - Collision dist: "<<collision_distance<<std::endl;                
                    
                    // Check if the collision is avoidable
                    if(collision_link_name!=end_effector_collision_link || isGoalInsideMeshBoundingBox(collision_object_name,goal_effector_position))
                    {
                        //Wait for object to clear path
                        std::cout<<"Unable to avoid collision with "<<collision_object_name<<", waiting until removed from currect position..."<<std::endl;
                        while(collision_distance>0 && collision_distance<collision_threshold)
                        {
                            //current_point = getCurrentTrajectoryPoint(trajectory_ptr,current_point);
                            collision_distance = getCollisionDistance(trajectory_ptr->points, collision_object_name, collision_link_name, collision_point, current_point);
                            //std::cout<<"Current point: "<<current_point<<" - Collision dist: "<<collision_distance<<std::endl;
                        }
                        std::cout<<"Collision cleared"<<std::endl;
                        auto stop_duration = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - stop_start_time);
                        execution_start_time = execution_start_time-stop_duration;
                    }
                    else
                    {
                        std::cout<<"Getting cartesian DMP..."<<std::endl;
                        std::vector<geometry_msgs::Pose> path;
             
                        getCartesianDmpPlan(trajectory_ptr, current_point, collision_object_name, cartesain_dmp_gamma, cartesain_dmp_beta, cartesain_dmp_k, cartesian_dmp_dt, cartesain_dmp_K, cartesain_dmp_D, cartesain_dmp_scale_m, cartesain_dmp_scale_n);

                        //trajectory_ptr =  boost::make_shared<trajectory_msgs::JointTrajectory>(trajectory);
                        std::cout<<"Start time"<<trajectory_ptr->points[0].time_from_start<<std::endl;
                        // TODO add custom validity check
                        //sleep(5);
                        current_point = 0;
                        execution_time = trajectory_ptr->points.back().time_from_start.toSec() -  trajectory_ptr->points.front().time_from_start.toSec();
                        goal_robot_state.setJointGroupPositions(move_group_ptr->getRobotModel()->getJointModelGroup("manipulator"),
                                                    trajectory_ptr->points.back().positions);
                        std::cout<<"New execution time: "<<execution_time<<std::endl;
                        
                        execution_start_time = std::chrono::high_resolution_clock::now();
                          
                        
                    }
                    robot_exectuion_status.data = 2;
                    robot_execution_status_publisher.publish(robot_exectuion_status);
                    std::cout<<"Moving..."<<std::endl;
                    // move_group_ptr->asyncExecute(getPlanFromJointTrajectory(trajectory_ptr, current_point));
                    moveit_msgs::MoveItErrorCodes error = move_group_ptr->asyncExecute(getPlanFromJointTrajectory(trajectory_ptr, current_point));
                    if(error.val!=1) 
                        std::cout<<"Error: "<<error.val<<std::endl;
                      
				}
            }
            else
				collision_count=0;

        }
        move_group_ptr->stop();
        std::cout<<"Execution successful"<<std::endl;
        robot_exectuion_status.data = 3;
        robot_execution_status_publisher.publish(robot_exectuion_status);
    };

    int getCurrentTrajectoryPoint(const trajectory_msgs::JointTrajectory::Ptr trajectory, int prev_point=0) 
    {
        // Initialize variables to store the closest point and its distance
        double min_distance = std::numeric_limits<double>::max();
        int closest_point = 0;
        robot_state::RobotState robot_state = *move_group_ptr->getCurrentState();
        const Eigen::Isometry3d& current_effector_pose = robot_state.getGlobalLinkTransform(end_effector_link);
        Eigen::Vector3d current_effector_position = current_effector_pose.translation();
        // Iterate through each point in the trajectory
        auto st = std::chrono::high_resolution_clock::now();
        double distance=-1;
        for (int i=prev_point;i<trajectory->points.size();i++) {
            // Convert the trajectory point to RobotState
            //st = std::chrono::high_resolution_clock::now();
            robot_state.setJointGroupPositions(move_group_ptr->getRobotModel()->getJointModelGroup("manipulator"),
                                                trajectory->points[i].positions);
            //std::cout<<"set state time: "<<std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - st).count()/1000.0<<std::endl;
            
            //st = std::chrono::high_resolution_clock::now();
            // Calculate the distance between the current state and the trajectory point
            //distance = move_group_ptr->getCurrentState()->distance(robot_state); //slow
            
            const Eigen::Isometry3d& traj_effector_pose = robot_state.getGlobalLinkTransform(end_effector_link);
            distance = (current_effector_position - traj_effector_pose.translation()).norm();
            //std::cout<<"Dist: "<<distance<<std::endl;
            //std::cout<<"get dist time: "<<std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - st).count()/1000.0<<std::endl;
            // Update the closest point if this point is closer
            if (distance < min_distance && distance>0) {
                min_distance = distance;
                closest_point = i;
            }
        }
        return closest_point;
    };


    std::vector<double> calculateCentroid(const std::vector<geometry_msgs::Point>& points) 
    {
        geometry_msgs::Point centroid;
        double numPoints = static_cast<double>(points.size());

        // Initialize sum to zero
        centroid.x = centroid.y = centroid.z = 0.0;

        // Calculate sum of coordinates
        for (const auto& point : points) {
            centroid.x += point.x;
            centroid.y += point.y;
            centroid.z += point.z;
        }

        // Calculate average
        centroid.x /= numPoints;
        centroid.y /= numPoints;
        centroid.z /= numPoints;

        return std::vector<double> {centroid.x,centroid.y,centroid.z};
    };

    bool isGoalInsideMeshBoundingBox(const std::string collision_object_name,const Eigen::Vector3d& point) 
    {
        moveit_msgs::CollisionObject collision_obj;
        planning_scene_ptr->getCollisionObjectMsg (collision_obj, collision_object_name);
        //std::cout<<collision_obj.pose<<std::endl; // 0 for meshes
        shape_msgs::Mesh mesh = collision_obj.meshes[0];
        
        std::vector<double> obstacle;

        for (const auto& vertex : mesh.vertices) {
            if (point.x() < vertex.x || point.x() > vertex.x ||
                point.y() < vertex.y || point.y() > vertex.y ||
                point.z() < vertex.z || point.z() > vertex.z) {
                return false;
            }
        }
        return true;
    };

    void getCartesianDmpPlan(trajectory_msgs::JointTrajectory::Ptr& trajectory_ptr, int current_point, std::string collision_object_name,
                             std::vector<double> gamma={200.0,500.0,15.0}, std::vector<double> beta={5.0,6.0}, std::vector<double> k={6.0,5.0,8.0},double dt=0.008, double K_gain=100.0, 
                             double D_gain=2.0 * pow(100,0.5), double cartesain_dmp_scale_m=0.0, double cartesain_dmp_scale_n=1.0, double tau=5, double t_0 = 0, std::vector<double> initial_velocities = {}, 
                             double seg_length=-1, int integrate_iter=1, std::vector<double> goal_thresh = {})
    {
        robot_state::RobotState robot_state = *move_group_ptr->getCurrentState();

        moveit_msgs::CollisionObject collision_obj;
        planning_scene_ptr->getCollisionObjectMsg (collision_obj, collision_object_name);
        //std::cout<<collision_obj.pose<<std::endl; // 0 for meshes
        shape_msgs::Mesh mesh = collision_obj.meshes[0];
        
        std::vector<double> obstacle;

        std::ofstream obstacleOutputFile("obstacle.txt");
        if (!obstacleOutputFile.is_open()) {
            std::cerr << "Error opening obstacle file" << std::endl;
        }
        else
        {
            for (const auto& point : mesh.vertices) {
                obstacle.push_back(point.x);
                obstacle.push_back(point.y);
                obstacle.push_back(point.z);

                obstacleOutputFile << point.x<<" "<<point.y<<" "<<point.z<<'\n';
            }
            // Close the file
            obstacleOutputFile.close();
        }

        //obstacle.push_back(calculateCentroid(mesh.vertices));
        //std::cout<<"Obstacle: "<<obstacle_centroid[0][0]<<" "<<obstacle_centroid[0][1]<<" "<<obstacle_centroid[0][2]<<std::endl;
        
        // Convert joint trajectory to Cartesian path
        trajectory_msgs::JointTrajectoryPoint trajectory_point;

        //Learn a DMP from colliding path
        std::vector<geometry_msgs::Pose> spaced_pose;
        dmp::DMPTraj demo_traj;
        /*
        const Eigen::Isometry3d& start_end_effector_pose = robot_state.getGlobalLinkTransform(end_effector_link); 
        Eigen::Vector3d start_end_effector_position = start_end_effector_pose.translation();
        Eigen::Vector3d start_end_effector_orientation = start_end_effector_pose.rotation().eulerAngles(2, 1, 0); 
        std::vector<double> pose={start_end_effector_position.x(),start_end_effector_position.y(),start_end_effector_position.z(),
                                    start_end_effector_orientation.x(),start_end_effector_orientation.y(),start_end_effector_orientation.z()};
        pt.positions=pose;
        demo_traj.points.push_back(pt);
        */

        std::ofstream pathOutputFile("path.txt");

        if (!pathOutputFile.is_open()) {
            std::cerr << "Error opening path file" << std::endl;
        }

        for (int i=current_point;i<trajectory_ptr->points.size(); i++)
        {
            trajectory_point = trajectory_ptr->points[i];
            // Set robot state based on joint values
            robot_state.setJointGroupPositions(move_group_ptr->getRobotModel()->getJointModelGroup("manipulator"),
                                            trajectory_point.positions);
            // Get end effector pose
            const Eigen::Isometry3d& end_effector_pose = robot_state.getGlobalLinkTransform(end_effector_link);
            
            Eigen::Vector3d end_effector_position = end_effector_pose.translation();
            Eigen::Vector3d end_effector_orientation = end_effector_pose.rotation().eulerAngles(2, 1, 0); 

            std::vector<double> pose={end_effector_position.x(),end_effector_position.y(),end_effector_position.z(),
                                      end_effector_orientation.x(),end_effector_orientation.y(),end_effector_orientation.z()};
            
            
            //if(computePoseVectorDistance(demo_traj.points.front().positions, pose)>0.01)
            //{  
                dmp::DMPPoint pt;
                pt.positions=pose;
                demo_traj.points.push_back(pt);
                demo_traj.times.push_back(dt*(i-current_point));
            //}
                // Write each point to the file
            pathOutputFile << end_effector_position.x()<<" "<<end_effector_position.y()<<" "<<end_effector_position.z()<<" "<<
                                      end_effector_orientation.x()<<" "<<end_effector_orientation.y()<<" "<<end_effector_orientation.z()<< '\n';
            
        }
        //std::cout<<"Smpled Traj size: "<<demo_traj.points.size()<<std::endl;
        //sleep(30);

        // Close the file
        pathOutputFile.close();

        dmp::LearnDMPFromDemo::Request l_req;
        l_req.demo = demo_traj;
        l_req.k_gains=std::vector<double>(6,K_gain);
        l_req.d_gains=std::vector<double>(6,D_gain);
        l_req.num_bases=int(cartesian_dmp_n_bases);
        
        dmp::LearnDMPFromDemo::Response l_res; 
        bool demo_learnt = learn_dmp_from_demo_client.call(l_req,l_res);


        dmp::SetActiveDMP::Request s_req;
        s_req.dmp_list=l_res.dmp_list;
        dmp::SetActiveDMP::Response s_res;
        bool active_dmp_set = set_active_dmp_client.call(s_req,s_res);


        /*
        robot_state=*move_group_ptr->getCurrentState();
        const Eigen::Isometry3d& end_effector_pose = robot_state.getGlobalLinkTransform(end_effector_link);
        Eigen::Vector3d end_effector_position = end_effector_pose.translation();
        Eigen::Vector3d end_effector_orientation = end_effector_pose.rotation().eulerAngles(2, 1, 0); 
        std::vector<double> x_0={end_effector_position.x(),end_effector_position.y(),end_effector_position.z(),
                                      end_effector_orientation.x(),end_effector_orientation.y(),end_effector_orientation.z()};
        */    

		robot_state = *move_group_ptr->getCurrentState();
		const Eigen::Isometry3d& end_effector_pose = robot_state.getGlobalLinkTransform(end_effector_link);
        Eigen::Vector3d end_effector_position = end_effector_pose.translation();
        Eigen::Vector3d end_effector_orientation = end_effector_pose.rotation().eulerAngles(2, 1, 0); 
        
        //std::vector<double> x_0 = {end_effector_position.x(),end_effector_position.y(),end_effector_position.z(),
        //                              end_effector_orientation.x(),end_effector_orientation.y(),end_effector_orientation.z()};
        std::vector<double> x_0 = demo_traj.points.front().positions;
        std::vector<double> goal = demo_traj.points.back().positions;
        std::vector<double> x_dot_0(x_0.size(), 0.0);
        tau = tau*2;
        
        if(goal_thresh.size()==0)
            goal_thresh.resize(x_0.size(), 0.01);
    
        dmp::GetDMPPlan::Request g_req;
        g_req.x_0=x_0; 
        g_req.x_dot_0=x_dot_0;
        g_req.t_0=t_0;
        g_req.goal=goal;
        g_req.goal_thresh=goal_thresh;
		g_req.seg_length=seg_length;
        g_req.tau=tau;
        g_req.dt=dt;
        g_req.integrate_iter=integrate_iter;
        g_req.beta=beta;
        g_req.gamma=gamma;
        g_req.k=k;
        g_req.scale_m=cartesain_dmp_scale_m;
        g_req.scale_n=cartesain_dmp_scale_n;
        g_req.obstacle=obstacle;
        dmp::GetDMPPlan::Response g_res; 
        
        auto start_time = std::chrono::high_resolution_clock::now();
        bool plan_received = get_dmp_plan_client.call(g_req, g_res);
        auto end_time = std::chrono::high_resolution_clock::now();
        double duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count()/1000.0;
        std::cout <<"DMP req execution time: "<<duration<< std::endl;

        std::vector<geometry_msgs::Pose> positions;
        std::vector<geometry_msgs::PoseStamped> stamped_positions;
        std::vector<geometry_msgs::Twist> velocities;
        geometry_msgs::PoseStamped stamped_pose;
        geometry_msgs::Twist vel;
        EigenSTL::vector_Isometry3d eigen_pose_path;
        for(int i=0;i<g_res.plan.points.size();i++)
        {   
            //std::cout<<g_res.plan.times[i]<<std::endl;
            geometry_msgs::Pose pose;
            pose.position.x=g_res.plan.points[i].positions[0];
            pose.position.y=g_res.plan.points[i].positions[1];
            pose.position.z=g_res.plan.points[i].positions[2];

            stamped_pose.pose=pose; // Ignore eef orientation to avoid path visualization error in Rviz
            stamped_positions.push_back(stamped_pose);

            std::vector<double> eulerZYX = {g_res.plan.points[i].positions[3],g_res.plan.points[i].positions[4],g_res.plan.points[i].positions[5]};
            Eigen::Quaterniond quaternion = eulerZYXToQauternion(eulerZYX);
            pose.orientation.x=quaternion.x();
            pose.orientation.y=quaternion.y();
            pose.orientation.z=quaternion.z();
            pose.orientation.w=quaternion.w();
            positions.push_back(pose);
            
            vel.linear.x=g_res.plan.points[i].velocities[0];
            vel.linear.y=g_res.plan.points[i].velocities[1];
            vel.linear.z=g_res.plan.points[i].velocities[2];
            vel.angular.x=g_res.plan.points[i].velocities[3];
            vel.angular.y=g_res.plan.points[i].velocities[4];
            vel.angular.z=g_res.plan.points[i].velocities[5];
            velocities.push_back(vel);
        }

        nav_msgs::Path imitated_path;
        imitated_path.header.frame_id = "base_link";
        imitated_path.poses = stamped_positions;
        cartesian_dmp_path_publisher.publish(imitated_path);


        std::cout<<"Plan from cartesian trajectory..."<<std::endl;
        double speed_scaling = 1/3.0;
        double initial_pose_th = 0.02;
        
        start_time = std::chrono::high_resolution_clock::now();
        trajectory_msgs::JointTrajectory trajectory = cartesianPathToJointTrajectory(positions, dt, initial_pose_th, speed_scaling);
        end_time = std::chrono::high_resolution_clock::now();
        duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count()/1000.0;
        std::cout <<"IK execution time: "<<duration<< std::endl;
        
        trajectory_ptr = boost::make_shared<trajectory_msgs::JointTrajectory>(trajectory);
        //sleep(60);
    };

    Eigen::Quaterniond eulerZYXToQauternion(std::vector<double> euler_angles)
    {
        Eigen::Quaterniond quaternion(Eigen::AngleAxisd(euler_angles[2], Eigen::Vector3d::UnitZ())
                                    * Eigen::AngleAxisd(euler_angles[1], Eigen::Vector3d::UnitY())
                                    * Eigen::AngleAxisd(euler_angles[0], Eigen::Vector3d::UnitX()));
        return quaternion;
    };
    
    void trajectoryCallback(const trajectory_msgs::JointTrajectory::ConstPtr& trajectory_msg)
    {
        std::cout<<"Trajectory received"<<std::endl;
        trajectory_msgs::JointTrajectoryPoint target_point = trajectory_msg->points.back();

        trajectory_msgs::JointTrajectory::Ptr trajectory_ptr = boost::const_pointer_cast<trajectory_msgs::JointTrajectory>(trajectory_msg);

        executePlanWithAvoidance(trajectory_ptr);
    };

private:
    // Ros publishers/subscribers
    ros::Subscriber dmp_trajectory_subscriber;
    ros::Publisher robot_execution_status_publisher;
    ros::Publisher cartesian_dmp_path_publisher;
    ros::Publisher cartesian_ik_path_publisher;

    
    planning_scene::PlanningScene *planning_scene_ptr;
    moveit::planning_interface::MoveGroupInterface *move_group_ptr; 

    bool debug;

    double collision_threshold;
    double goal_threshold;
    double collision_check_sampling;

    std::string end_effector_link;
    std::string end_effector_collision_link;

    ros::ServiceClient get_dmp_plan_client;
    ros::ServiceClient learn_dmp_from_demo_client;
    ros::ServiceClient set_active_dmp_client;
    ros::ServiceClient scene_client;

    // Cartesian DMP
    double cartesian_dmp_speed_scale;
    double cartesian_dmp_dt;
    double cartesain_dmp_K;
    double cartesain_dmp_D;
    double cartesain_dmp_scale_m;
    double cartesain_dmp_scale_n; 
    std::vector<double> cartesain_dmp_gamma;
    std::vector<double> cartesain_dmp_beta;
    std::vector<double> cartesain_dmp_k;
    int cartesian_dmp_n_bases;
    double ik_jump_threshold_factor;
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
