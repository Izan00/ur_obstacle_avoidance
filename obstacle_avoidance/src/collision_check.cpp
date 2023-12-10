#include <ros/ros.h>
#include "obstacle_avoidance.h"
#include "obstacle_avoidance/AvoidanceExecute.h"

#include <std_msgs/Int32.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/RobotState.h>

#include <moveit/robot_state/cartesian_interpolator.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include<dmp/GetDMPPlan.h> //GetDMPPlanRequest,LearnDMPFromDemo, LearnDMPFromDemoRequest, SetActiveDMP, SetActiveDMPRequest
#include<dmp/DMPTraj.h>
#include<dmp/DMPData.h>
#include<dmp/DMPPoint.h>

#include"cartesian_dmp/DMPRequest.h"
#include <eigen_conversions/eigen_msg.h>

#include<cmath>
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
        std::string robot_execution_status_topic;

        // Parameters update
        nh.param<std::string>("trajectory_sub_topic", trajectory_sub_topic, "/robot_global_trajectory");
        nh.param<std::string>("avoidance_execute_sub_topic", avoidance_execute_sub_topic, "/avoidance_execute");
        nh.param<std::string>("robot_execution_status_topic", robot_execution_status_topic, "/robot_execution_status");
        
        nh.param<bool>("debug", debug, false);

        nh.param<double>("collision_threshold", collision_threshold, 0.3); //Set the collisin check horizont distance in m
        nh.param<int>("goal_threshold", goal_threshold, 0.1); // Joints L2 norm in m

        nh.param<double>("cartesian_dmp_speed_scale", cartesian_dmp_speed_scale, 0.5); 
        nh.param<double>("cartesian_dmp_dt", cartesian_dmp_dt, 0.1); 
        nh.param<double>("cartesain_dmp_gamma", cartesain_dmp_gamma, 1000.0); 
        nh.param<double>("cartesain_dmp_beta", cartesain_dmp_beta, 4.0/M_PI); 
        nh.param<int>("cartesian_dmp_n_weights_per_dim", cartesian_dmp_n_weights_per_dim, 100); 
        nh.param<double>("ik_jump_threshold_factor", ik_jump_threshold_factor, 0.05);
        
        nh.param<std::string>("end_effector_link", end_effector_link, "rg2_eef_link");
        nh.param<std::string>("end_effector_collision_link", end_effector_collision_link, "rg2_body_link");
        
        // Get moveit planning object
        move_group_ptr = new moveit::planning_interface::MoveGroupInterface("manipulator");
        planning_scene_ptr = new planning_scene::PlanningScene(move_group_ptr->getRobotModel());

        move_group_ptr->setEndEffectorLink(end_effector_link);

        // Ros publishers/subscribers
        trajectory_subscriber = nh.subscribe(trajectory_sub_topic, 10, &CollsionCheck::trajectoryCallback, this);
        execute_subscriber = nh.subscribe(avoidance_execute_sub_topic, 10, &CollsionCheck::executeCallback, this);
        
        robot_execution_status_publisher = nh.advertise<std_msgs::Int32>(robot_execution_status_topic, 10);

        // Create an action client for DMP
        get_dmp_plan_client = nh.serviceClient<dmp::GetDMPPlan>("get_dmp_plan");

        // Create an action client for DMP
        get_cartesian_dmp_plan_client = nh.serviceClient<cartesian_dmp::DMPRequest>("get_cartesian_dmp");

        // Create an action client for planning scene
        scene_client = nh.serviceClient<moveit_msgs::GetPlanningScene>("get_planning_scene");

        ROS_INFO("Node started");

    };

    double getCollisionDistance(const std::vector<trajectory_msgs::JointTrajectoryPoint>& trajectory_points, std::string& collision_object, int path_start=0)
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
            // Set the robot state to the current trajectory point
            robot_state.setJointGroupPositions(move_group_ptr->getRobotModel()->getJointModelGroup("manipulator"),
                                            trajectory_points[i].positions);

            // Update the Planning Scene with the current robot state
            planning_scene_ptr->setCurrentState(robot_state);

            // Collision checking
            // Clear previous collision responses
            collision_result.clear();
            planning_scene_ptr->checkCollision(collision_request, collision_result);

            // If collision occurs
            if (collision_result.collision) 
            {
                planning_scene_ptr->getCollidingPairs(contacts);
                for (const auto& contact : contacts) {
                    const std::string& object1 = contact.first.first;
                    const std::string& object2 = contact.first.second;
                    //std::cout<<"Collision between: " << object1 << " and " << object2<<std::endl;
                    if(object1==end_effector_collision_link || object2==end_effector_collision_link)
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
                            }
                        }
                        //std::cout<<"Collision_distance: "<< collision_distance<<std::endl;
                        // Stop the timer
                        auto end_time = std::chrono::high_resolution_clock::now();
                        // Calculate the duration
                        double duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count()/1000.0;
                        // Print the duration
                        //std::cout <<"Execution time: "<<duration << std::endl;
                        
                        if(object1==end_effector_collision_link)
                            collision_object=object2;
                        else
                            collision_object=object1;

                        return collision_distance;
                        //return false;
                    }
                }
            }
        }

         // Stop the timer
        auto end_time = std::chrono::high_resolution_clock::now();
        // Calculate the duration
        double duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count()/1000.0;
        // Print the duration
        //std::cout <<"Execution time: "<<duration<< std::endl;

        // If no collisions occurred in any trajectory point, the path is valid
        return collision_distance;
        //return true;
    };

    std::vector<geometry_msgs::Pose> getCartesianDMP(trajectory_msgs::JointTrajectory& trajectory, int current_point, std::string collision_object_name,
                                                    int n_weights_per_dim = 100, double gamma=1000.0, double beta=4.0/M_PI, double dt=0.1)
    {

        robot_state::RobotState robot_state = *move_group_ptr->getCurrentState();

        moveit_msgs::CollisionObject collision_obj;
        planning_scene_ptr->getCollisionObjectMsg (collision_obj, collision_object_name);
        //std::cout<<collision_obj.pose<<std::endl; // 0 for meshes
        shape_msgs::Mesh mesh = collision_obj.meshes[0];
        std::vector<geometry_msgs::Point> obstacles = mesh.vertices;
        
        // Convert joint trajectory to Cartesian path
        std::vector<geometry_msgs::Pose> cartesian_path;
        trajectory_msgs::JointTrajectoryPoint trajectory_point;

        // Get end effector pose
        //geometry_msgs::PoseStamped end_effector_pose = move_group_ptr->getCurrentPose();
        //cartesian_path.push_back(end_effector_pose.pose); 

        for (int i=current_point;i<trajectory.points.size(); i++)
        {
            trajectory_point = trajectory.points[i];
            // Set robot state based on joint values
            robot_state.setJointGroupPositions(move_group_ptr->getRobotModel()->getJointModelGroup("manipulator"),
                                            trajectory_point.positions);
            // Get end effector pose
            const Eigen::Isometry3d& end_effector_pose = robot_state.getGlobalLinkTransform(end_effector_link);

            // Convert Eigen pose to geometry_msgs::Pose
            geometry_msgs::Pose pose_msg;
            tf::poseEigenToMsg(end_effector_pose, pose_msg);

            // Add the pose to the Cartesian path
            cartesian_path.push_back(pose_msg);
        }

        cartesian_dmp::DMPRequest::Request request;
        request.demo = cartesian_path;
        request.gamma = gamma;
        request.beta = beta;
        request.dt = dt;
        request.n_weights_per_dim = n_weights_per_dim;
        request.execution_time = trajectory.points.back().time_from_start.toSec() -  trajectory.points[current_point].time_from_start.toSec();
        request.obstacles = obstacles;

        cartesian_dmp::DMPRequest::Response response;
        get_cartesian_dmp_plan_client.call(request,response);

        return response.path;
    };
    
    moveit::planning_interface::MoveGroupInterface::Plan getPlanFromJointTrajectory(const trajectory_msgs::JointTrajectory::ConstPtr trajectory)
    {
        moveit_msgs::RobotState robot_state_msg;
        moveit::core::robotStateToRobotStateMsg(*move_group_ptr->getCurrentState(), robot_state_msg);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_.joint_trajectory = *trajectory;
        plan.start_state_  = robot_state_msg;
        trajectory_msgs::JointTrajectoryPoint start_point;
        start_point.positions = robot_state_msg.joint_state.position;
        start_point.velocities = std::vector<double>(6,0);
        plan.trajectory_.joint_trajectory.points.insert(plan.trajectory_.joint_trajectory.points.begin(), start_point);

        return plan;    
    };

    moveit::planning_interface::MoveGroupInterface::Plan getPlanFromCartesianTrajectory(std::vector<geometry_msgs::Pose>& path)
    {
        moveit_msgs::RobotTrajectory trajectory;
        moveit_msgs::MoveItErrorCodes *error;
        std::cout<<"Computing"<<std::endl;
        /*
        fraction = move_group_ptr->computeCartesianPath(path, 0.01, 0.0, trajectory, true, error);
        std::cout<<"Plan fraction: "<<fraction<<std::endl;
        */

        EigenSTL::vector_Isometry3d waypoints(path.size());

        std::cout<<"Path size: "<<path.size()<<std::endl;
        for (std::size_t i = 0; i < path.size(); ++i)
            tf::poseMsgToEigen(path[i], waypoints[i]);

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

        traj.clear();
        traj.push_back(std::make_shared<moveit::core::RobotState>(start_state));

        int steps = waypoints.size();
        std::cout<<"Computing path IK..."<<std::endl;
        double dist_prev_point=0;
        bool prev_added=false;
        for (std::size_t i = 1; i <= steps; ++i)
        {
            if (start_state.setFromIK(group, waypoints[i], link->getName(), consistency_limits, 0.0, constraint_fn, options))
            {
                if(traj.size()>0)
                    dist_prev_point = std::make_shared<moveit::core::RobotState>(start_state)->distance(*traj.back(), group);
                //if(dist_prev_point<0.02)
                traj.push_back(std::make_shared<moveit::core::RobotState>(start_state));
                //std::cout<<"Dist: "<<dist_prev_point<<std::endl;
            }
            //std::cout<<i<<"/"<<steps<<" - Traj size: "<<traj.size()<<std::endl;
        }
        //moveit::core::CartesianInterpolator::checkJointSpaceJump(group, traj, jump_threshold);
        std::cout<<"IK path size: "<<traj.size()<<std::endl;

        robot_trajectory::RobotTrajectory rt(start_state.getRobotModel(), "manipulator");
        for (std::size_t i = 0; i < traj.size(); ++i)
            rt.addSuffixWayPoint(traj[i], 0.0);

        // Add time to trajectory
        trajectory_processing::IterativeParabolicTimeParameterization time_param;
        time_param.computeTimeStamps(rt, cartesian_dmp_speed_scale,cartesian_dmp_speed_scale); // scale speed for sim

        // Extract trajectrory
        moveit_msgs::RobotTrajectory solution;
        rt.getRobotTrajectoryMsg(solution);
        
        moveit_msgs::RobotState robot_state_msg;
        moveit::core::robotStateToRobotStateMsg(*move_group_ptr->getCurrentState(), robot_state_msg);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        //plan.trajectory_.joint_trajectory = trajectory.joint_trajectory;
        plan.trajectory_ = solution;
        plan.start_state_  = robot_state_msg;
        trajectory_msgs::JointTrajectoryPoint start_point;
        start_point.positions = robot_state_msg.joint_state.position;
        start_point.velocities = std::vector<double>(6,0);
        
        return plan;
    };

    void executeCallback(const obstacle_avoidance::AvoidanceExecute::ConstPtr& execute_msg){
        //Generate a plan...            
        std::vector<double> initial_pose =  execute_msg->start_point.positions;
        double tau =  execute_msg->tau;
        double dt =  execute_msg->dt;
        std::vector<double> goal_pose =  execute_msg->target_point.positions;

        trajectory_msgs::JointTrajectory trajectory;

        getDmpPlan(trajectory, initial_pose,goal_pose,tau,dt);
        
        trajectory_msgs::JointTrajectory::ConstPtr trajectory_ptr =  boost::make_shared<trajectory_msgs::JointTrajectory>(trajectory);
        
        std::cout<<"Starting plan..."<<std::endl;
  
        std_msgs::Int32 robot_exectuion_status; //0-stopped 1-running 2-success 3-failed
        robot_exectuion_status.data = 1;
        robot_execution_status_publisher.publish(robot_exectuion_status);

        double execution_time = trajectory.points.back().time_from_start.toSec() -  trajectory.points.front().time_from_start.toSec();
        std::cout<<"Execution time: "<<execution_time<<std::endl;
        double collision_distance = -1;
        std::string collision_object = "";
        
        // Get goal state
        robot_state::RobotState goal_robot_state = *move_group_ptr->getCurrentState();
        goal_robot_state.setJointGroupPositions(move_group_ptr->getRobotModel()->getJointModelGroup("manipulator"),
                                            trajectory.points.back().positions);
        robot_state::RobotState current_state = *move_group_ptr->getCurrentState();
        double goal_dist = current_state.distance(goal_robot_state);
    
        // Start movement
        move_group_ptr->asyncExecute(getPlanFromJointTrajectory(trajectory_ptr));
        //moveit_msgs::MoveItErrorCodes error = move_group_ptr->asyncExecute(getPlanFromJointTrajectory(trajectory_ptr));
        std::cout<<"Moving..."<<std::endl;
        auto execution_start_time = std::chrono::high_resolution_clock::now();
        int current_point = 0;
        
        while(current_point<trajectory.points.size()-1 && goal_dist>goal_threshold)
        {  
            robot_state::RobotState current_state = *move_group_ptr->getCurrentState();
            goal_dist = current_state.distance(goal_robot_state);
            
            // Calculate the duration
            double execution_duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - execution_start_time).count()/1000.0;
            if(execution_duration> execution_time + 10) // 5s threshold
            {   
                std::cout<<"Execution error"<<std::endl;
                robot_exectuion_status.data = 3;
                robot_execution_status_publisher.publish(robot_exectuion_status);
                move_group_ptr->stop();
                return;
            }
            current_point = getCurrentTrajectoryPoint(trajectory.points);
            collision_distance = getCollisionDistance(trajectory.points, collision_object,current_point);            
            //std::cout<<"Current point: "<<current_point<<"/"<<trajectory.points.size()<<" - Collision dist: "<<collision_distance<<std::endl;
            if(debug)
                std::cout<<"Current point: "<<current_point<<"/"<<trajectory.points.size()<<" - Collision dist: "<<collision_distance<<" - Duration: "<<execution_duration<<" -  Goal dist: "<<goal_dist<<std::endl;
            if(collision_distance>0 && collision_distance<collision_threshold)
            {
                std::cout<<"Stoped"<<std::endl;
                move_group_ptr->stop(); // has some delay btween sent and stoped
                auto stop_start_time = std::chrono::high_resolution_clock::now();
                robot_exectuion_status.data = 0;
                robot_execution_status_publisher.publish(robot_exectuion_status);
                sleep(1);
                current_point = getCurrentTrajectoryPoint(trajectory.points);
                collision_distance = getCollisionDistance(trajectory.points, collision_object,current_point);            
                //std::cout<<"Current point: "<<current_point<<"/"<<trajectory.points.size()<<" - Collision dist: "<<collision_distance<<std::endl;
                std::cout<<"Current point: "<<current_point<<"/"<<trajectory.points.size()<<" - Collision dist: "<<collision_distance<<" - Duration: "<<execution_duration <<" -  Goal dist: "<<goal_dist<<std::endl;
            
                /*
                //Wait for object to clear path
                while(collision_distance>0 && collision_distance<collision_threshold)
                {
                    current_point = getCurrentTrajectoryPoint(trajectory.points);
                    collision_distance = getCollisionDistance(trajectory.points, collision_object, current_point);
                    std::cout<<"Current point: "<<current_point<<" - Collision dist: "<<collision_distance<<std::endl;
                }
                */
                std::cout<<"Getting cartesian DMP..."<<std::endl;
                std::vector<geometry_msgs::Pose> path;
                path = getCartesianDMP(trajectory, current_point, collision_object, cartesian_dmp_n_weights_per_dim, cartesain_dmp_gamma, cartesain_dmp_beta, cartesian_dmp_dt);

                std::cout<<"Plan from cartesian trajectory..."<<std::endl;
                moveit::planning_interface::MoveGroupInterface::Plan plan = getPlanFromCartesianTrajectory(path);
                /*
                // TODO add custom validity check
                if (!valid)
                {   
                    std::cout<<"Cartesian to joint conversion error"<<std::endl;
                    robot_exectuion_status.data = 3;
                    robot_execution_status_publisher.publish(robot_exectuion_status);
                    move_group_ptr->stop();
                    return;
                }
                */
                trajectory.points.clear();
                trajectory = plan.trajectory_.joint_trajectory;
                current_point = 0;
                execution_time = trajectory.points.back().time_from_start.toSec() -  trajectory.points.front().time_from_start.toSec();
                goal_robot_state.setJointGroupPositions(move_group_ptr->getRobotModel()->getJointModelGroup("manipulator"),
                                            trajectory.points.back().positions);
                std::cout<<"New execution time: "<<execution_time<<std::endl;
                std::cout<<"Moving..."<<std::endl;
                move_group_ptr->asyncExecute(plan);
                //auto stop_duration = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - stop_start_time);
                execution_start_time = std::chrono::high_resolution_clock::now();
            }

        }
        std::cout<<"Execution successful"<<std::endl;
        robot_exectuion_status.data = 2;
        robot_execution_status_publisher.publish(robot_exectuion_status);
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
        //for(int i=0; i<10;i++)
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
    ros::Publisher robot_execution_status_publisher;
    
    planning_scene::PlanningScene *planning_scene_ptr;
    moveit::planning_interface::MoveGroupInterface *move_group_ptr; 

    bool debug;

    double collision_threshold;
    int goal_threshold;

    std::string end_effector_link;
    std::string end_effector_collision_link;

    ros::ServiceClient get_dmp_plan_client;
    ros::ServiceClient get_cartesian_dmp_plan_client;
    ros::ServiceClient scene_client;

    // Cartesian DMP
    double cartesian_dmp_speed_scale;
    double cartesian_dmp_dt;
    double cartesain_dmp_gamma;
    double cartesain_dmp_beta;
    int cartesian_dmp_n_weights_per_dim;
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
