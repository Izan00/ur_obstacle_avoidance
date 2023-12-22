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
#include <nav_msgs/Path.h>
#include <moveit/robot_state/cartesian_interpolator.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <eigen_conversions/eigen_msg.h>

#include<dmp/GetDMPPlan.h> 
#include<dmp/LearnDMPFromDemo.h> 
#include<dmp/SetActiveDMP.h> 
#include<dmp/DMPTraj.h>
#include<dmp/DMPData.h>
#include<dmp/DMPPoint.h>

#include <tf2_eigen/tf2_eigen.h>

#include<cmath>
#include<chrono>
#include<thread>
#include <algorithm>

namespace
{
void ikCallbackFnAdapter(robot_state::RobotState* state, const robot_state::JointModelGroup* group,
                        const robot_state::GroupStateValidityCallbackFn& constraint, const geometry_msgs::Pose& /*unused*/,
                        const std::vector<double>& ik_sol, moveit_msgs::MoveItErrorCodes& error_code)
{
    const std::vector<unsigned int>& bij = group->getKinematicsSolverJointBijection();
    std::vector<double> solution(bij.size());
    for (std::size_t i = 0; i < bij.size(); ++i)
        solution[bij[i]] = ik_sol[i];
    if (constraint(state, group, &solution[0]))
        error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
    else
        error_code.val = moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION;
}
};

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
        
        nh.param<bool>("debug", debug, true);

        nh.param<double>("collision_threshold", collision_threshold, 0.3); //Set the collisin check horizont distance in m
        nh.param<int>("goal_threshold", goal_threshold, 0.05); // Joints L2 norm in m

        nh.param<double>("cartesian_dmp_speed_scale", cartesian_dmp_speed_scale, 1.0); 
        nh.param<double>("cartesian_dmp_dt", cartesian_dmp_dt, 0.01); 
        nh.param<double>("cartesain_dmp_gamma", cartesain_dmp_gamma, 1000.0); 
        nh.param<double>("cartesain_dmp_beta", cartesain_dmp_beta, 20.0/M_PI); 
        nh.param<double>("cartesain_dmp_k", cartesain_dmp_k, 100); 
        nh.param<double>("cartesain_dmp_d", cartesain_dmp_d, 2.0*pow(cartesain_dmp_k,0.5)); 
        nh.param<int>("cartesian_dmp_n_weights_per_dim", cartesian_dmp_n_weights_per_dim, 100);
        nh.param<int>("cartesian_dmp_n_bases", cartesian_dmp_n_bases, 100);  
        nh.param<double>("ik_jump_threshold_factor", ik_jump_threshold_factor, 0.02);
        
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
        cartesian_dmp_path_publisher = nh.advertise<nav_msgs::Path>("/imitated_path_avoidance",10);
        cartesian_ik_path_publisher = nh.advertise<nav_msgs::Path>("/true_imitated_path_avoidance",10);


        // Create action clients for DMP
        get_dmp_plan_client = nh.serviceClient<dmp::GetDMPPlan>("get_dmp_plan");
        get_cartesian_dmp_plan_client = nh.serviceClient<dmp::GetDMPPlan>("cartesian/get_dmp_plan");
        learn_dmp_from_demo_client = nh.serviceClient<dmp::LearnDMPFromDemo>("cartesian/learn_dmp_from_demo");
        set_active_dmp_client = nh.serviceClient<dmp::SetActiveDMP>("cartesian/set_active_dmp");

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

    bool setFromIK(robot_state::RobotState robot_state,const robot_model::JointModelGroup* jmg, const EigenSTL::vector_Isometry3d& poses_in,
                    const std::vector<std::string>& tips_in,
                    const std::vector<std::vector<double> >& consistency_limit_sets, double timeout,
                    const robot_state::GroupStateValidityCallbackFn& constraint,
                    const kinematics::KinematicsQueryOptions& options)
    {
        // Error check
        if (poses_in.size() != tips_in.size())
        {
            ROS_ERROR("Number of poses must be the same as number of tips");
            return false;
        }

        // Load solver
        const kinematics::KinematicsBaseConstPtr& solver = jmg->getSolverInstance();

        // Check if this jmg has a solver
        bool valid_solver = true;
        if (!solver)
        {
            valid_solver = false;
        }
        // Check if this jmg's IK solver can handle multiple tips (non-chain solver)
        else if (poses_in.size() > 1)
        {
            std::string error_msg;
            if (!solver->supportsGroup(jmg, &error_msg))
            {
            // skirt around clang-diagnostic-potentially-evaluated-expression
            const kinematics::KinematicsBase& solver_ref = *solver;
            ROS_ERROR("Kinematics solver %s does not support joint group %s.  Error: %s",
                            typeid(solver_ref).name(), jmg->getName().c_str(), error_msg.c_str());
            valid_solver = false;
            }
        }

        if (!valid_solver)
        {
        
            ROS_ERROR("No kinematics solver instantiated for group '%s'", jmg->getName().c_str());
            return false;
        }

        // Check that no, or only one set of consistency limits has been passed in, and choose that one
        std::vector<double> consistency_limits;
        if (consistency_limit_sets.size() > 1)
        {
            ROS_ERROR("Invalid number (%zu) of sets of consistency limits for a setFromIK request "
                            "that is being solved by a single IK solver",
                            consistency_limit_sets.size());
            return false;
        }
        else if (consistency_limit_sets.size() == 1)
            consistency_limits = consistency_limit_sets[0];

        // ensure RobotState is up-to-date before employing it in the IK solver
        robot_state.RobotState::update(false);

        const std::vector<std::string>& solver_tip_frames = solver->getTipFrames();

        // Track which possible tips frames we have filled in so far
        std::vector<bool> tip_frames_used(solver_tip_frames.size(), false);

        // Create vector to hold the output frames in the same order as solver_tip_frames
        std::vector<geometry_msgs::Pose> ik_queries(solver_tip_frames.size());

        // Bring each pose to the frame of the IK solver
        for (std::size_t i = 0; i < poses_in.size(); ++i)
        {
            // Make non-const
            Eigen::Isometry3d pose = poses_in[i];
            std::string pose_frame = tips_in[i];

            // Remove extra slash
            if (!pose_frame.empty() && pose_frame[0] == '/')
            pose_frame = pose_frame.substr(1);

            // bring the pose to the frame of the IK solver
            if (!robot_state.setToIKSolverFrame(pose, solver))
            return false;

            // try all of the solver's possible tip frames to see if they match with any of the passed-in pose tip frames
            bool found_valid_frame = false;
            std::size_t solver_tip_id;  // our current index
            for (solver_tip_id = 0; solver_tip_id < solver_tip_frames.size(); ++solver_tip_id)
            {
            // Check if this tip frame is already accounted for
            if (tip_frames_used[solver_tip_id])
                continue;  // already has a pose

            // check if the tip frame can be transformed via fixed transforms to the frame known to the IK solver
            std::string solver_tip_frame = solver_tip_frames[solver_tip_id];

            // remove the frame '/' if there is one, so we can avoid calling Transforms::sameFrame() which may copy strings
            // more often that we need to
            if (!solver_tip_frame.empty() && solver_tip_frame[0] == '/')
                solver_tip_frame = solver_tip_frame.substr(1);

            if (pose_frame != solver_tip_frame)
            {
                Eigen::Isometry3d pose_parent_to_frame;
                auto* pose_parent = robot_state.RobotState::getRigidlyConnectedParentLinkModel(pose_frame, &pose_parent_to_frame, jmg);
                if (!pose_parent)
                {
                ROS_ERROR_STREAM("Pose frame '" << pose_frame << "' does not exist.");
                return false;
                }
                Eigen::Isometry3d tip_parent_to_tip;
                auto* tip_parent = robot_state.RobotState::getRigidlyConnectedParentLinkModel(solver_tip_frame, &tip_parent_to_tip, jmg);
                if (!tip_parent)
                {
                ROS_ERROR_STREAM("Solver tip frame '" << solver_tip_frame << "' does not exist.");
                return false;
                }
                if (pose_parent == tip_parent)
                {
                // transform goal pose as target for solver_tip_frame (instead of pose_frame)
                pose = pose * pose_parent_to_frame.inverse() * tip_parent_to_tip;
                found_valid_frame = true;
                break;
                }
            }
            else
            {
                found_valid_frame = true;
                break;
            }
            }  // end for solver_tip_frames

            // Make sure one of the tip frames worked
            if (!found_valid_frame)
            {
            ROS_ERROR("Cannot compute IK for query %zu pose reference frame '%s'", i, pose_frame.c_str());
            // Debug available tip frames
            std::stringstream ss;
            for (solver_tip_id = 0; solver_tip_id < solver_tip_frames.size(); ++solver_tip_id)
                ss << solver_tip_frames[solver_tip_id] << ", ";
            ROS_ERROR( "Available tip frames: [%s]", ss.str().c_str());
            return false;
            }

            // Remove that tip from the list of available tip frames because each can only have one pose
            tip_frames_used[solver_tip_id] = true;

            // Convert Eigen pose to geometry_msgs pose
            geometry_msgs::Pose ik_query;
            ik_query = tf2::toMsg(pose);

            // Save into vectors
            ik_queries[solver_tip_id] = ik_query;
        }  // end for poses_in

        // Create poses for all remaining tips a solver expects, even if not passed into this function
        for (std::size_t solver_tip_id = 0; solver_tip_id < solver_tip_frames.size(); ++solver_tip_id)
        {
            // Check if this tip frame is already accounted for
            if (tip_frames_used[solver_tip_id])
            continue;  // already has a pose

            // Process this tip
            std::string solver_tip_frame = solver_tip_frames[solver_tip_id];

            // remove the frame '/' if there is one, so we can avoid calling Transforms::sameFrame() which may copy strings more
            // often that we need to
            if (!solver_tip_frame.empty() && solver_tip_frame[0] == '/')
            solver_tip_frame = solver_tip_frame.substr(1);

            // Get the pose of a different EE tip link
            Eigen::Isometry3d current_pose = robot_state.RobotState::getGlobalLinkTransform(solver_tip_frame);

            // bring the pose to the frame of the IK solver
            if (!robot_state.RobotState::setToIKSolverFrame(current_pose, solver))
            return false;

            // Convert Eigen pose to geometry_msgs pose
            geometry_msgs::Pose ik_query;
            ik_query = tf2::toMsg(current_pose);

            // Save into vectors - but this needs to be ordered in the same order as the IK solver expects its tip frames
            ik_queries[solver_tip_id] = ik_query;

            // Remove that tip from the list of available tip frames because each can only have one pose
            tip_frames_used[solver_tip_id] = true;
        }

        // if no timeout has been specified, use the default one
        if (timeout < std::numeric_limits<double>::epsilon())
            timeout = jmg->getDefaultIKTimeout();

        // set callback function
        robot_state::RobotState *robot_state_ptr = &robot_state;
        kinematics::KinematicsBase::IKCallbackFn ik_callback_fn;
        if (constraint)
            ik_callback_fn = [robot_state_ptr, jmg, constraint](const geometry_msgs::Pose& pose, const std::vector<double>& joints,
                                                    moveit_msgs::MoveItErrorCodes& error_code) {
            ikCallbackFnAdapter(robot_state_ptr, jmg, constraint, pose, joints, error_code);
            };

        // Bijection
        const std::vector<unsigned int>& bij = jmg->getKinematicsSolverJointBijection();

        std::vector<double> initial_values;
        robot_state.RobotState::copyJointGroupPositions(jmg, initial_values);
        std::vector<double> seed(bij.size());
        for (std::size_t i = 0; i < bij.size(); ++i)
            seed[i] = initial_values[bij[i]];

        // compute the IK solution
        std::vector<double> ik_sol;
        moveit_msgs::MoveItErrorCodes error;
        
        if (solver->searchPositionIK(ik_queries, seed, timeout, consistency_limits, ik_sol, ik_callback_fn, error, options,
                                    &robot_state))
        {
            std::vector<double> solution(bij.size());
            for (std::size_t i = 0; i < bij.size(); ++i)
            solution[bij[i]] = ik_sol[i];
            robot_state.RobotState::setJointGroupPositions(jmg, solution);
            return true;
        }
        return false;
    };

    trajectory_msgs::JointTrajectory cartesianPathToJointTrajectory(std::vector<geometry_msgs::Pose>& positions, std::vector<geometry_msgs::Twist>& veolicities, double dt, std::string method="pose")
    {
        moveit_msgs::RobotTrajectory trajectory;
        moveit_msgs::MoveItErrorCodes *error;
        std::cout<<"Positions size: "<<positions.size()<<std::endl;

        /*
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

        //double fraction = move_group_ptr->computeCartesianPath(positions, 0.01, 0.0, trajectory, true, error);
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
        bool success;
        int last_valid_ik = 0;
        robot_trajectory::RobotTrajectory rt(start_state.getRobotModel(), "manipulator");

        int last_bound=0;        
        for(std::size_t i = 0; i <= steps; ++i)
        {

            if(method=="vel")
            {
                success = start_state.setFromDiffIK(group, veolicities[i], link->getName(), dt, constraint_fn);
            }
            else if(method=="eigen")
            {   
                Eigen::Isometry3d eigen_pose;
                tf::poseMsgToEigen(positions[i], eigen_pose);
                success = start_state.setFromIK(group, eigen_pose, link->getName(), consistency_limits, 0.1, constraint_fn, options);
            }
            else if(method=="eigen2")
            {
                std::vector<std::vector<double>> consistency_limits_vector;
                consistency_limits_vector.push_back(consistency_limits);
                double timeout=0.02;
                std::string tip_link=link->getName();
                std::vector<std::string> tip_link_vector;
                tip_link_vector.push_back(tip_link);
                Eigen::Isometry3d eigen_pose;
                tf::poseMsgToEigen(positions[i], eigen_pose);
                EigenSTL::vector_Isometry3d eigen_pose_vector;
                eigen_pose_vector.push_back(eigen_pose);
                //success = start_state.RobotState::setFromIK(group, eigen_pose_vector, tip_link_vector, consistency_limits_vector, timeout, constraint_fn, options);
                success = setFromIK(start_state, group, eigen_pose_vector, tip_link_vector, consistency_limits_vector, timeout, constraint_fn, options);
                //std::cout<<"Point "<<i<<": "<<success<<std::endl;
            }
            else
            {
                //geometry_msgs::Pose pose =  positions[i];
                success = start_state.setFromIK(group, positions[i], link->getName(), 0.0, constraint_fn, options);
            }
            if(success)
            {
                if(traj.size()>0)
                    dist_prev_point = std::make_shared<moveit::core::RobotState>(start_state)->distance(*traj.back(), group);
                //if(dist_prev_point<0.02)
                traj.push_back(std::make_shared<moveit::core::RobotState>(start_state));
                //std::cout<<"Dist: "<<dist_prev_point<<std::endl;
                //rt.addSuffixWayPoint(traj.back(), 0.0); // Need time to be set below
                double current_dt = (i-last_valid_ik)*dt;
                if(current_dt==0)
                    current_dt=dt;
                rt.addSuffixWayPoint(traj.back(), current_dt);
                last_valid_ik=i;
            }

            if(traj.size()>10){
            moveit::core::CartesianInterpolator::checkJointSpaceJump(group, traj, jump_threshold);
            //int current_last_bound = traj.size();
            start_state = *traj.back();
            }
            std::cout<<i<<"/"<<steps<<" - Traj size: "<<traj.size()<<std::endl;
        }
        int ik_size = traj.size();
        std::cout<<"IK path size: "<<ik_size<<std::endl;
        
        // Jumps check
        std::vector<moveit::core::RobotStatePtr> traj_bounded = traj;
        moveit::core::CartesianInterpolator::checkJointSpaceJump(group, traj_bounded, jump_threshold);
        int ik_jumps_size = traj_bounded.size();
        std::cout<<"Bounded IK path size: "<<ik_jumps_size<<std::endl;
        
        /*
        // Add time to trajectory
        trajectory_processing::IterativeParabolicTimeParameterization time_param;
        if(method=="pose" || method=="eigen")
            time_param.computeTimeStamps(rt, cartesian_dmp_speed_scale,cartesian_dmp_speed_scale); // scale speed for sim
        */

        // Extract trajectrory
        moveit_msgs::RobotTrajectory solution;
        rt.getRobotTrajectoryMsg(solution);
        
        return solution.joint_trajectory;
    };

    void executeCallback(const obstacle_avoidance::AvoidanceExecute::ConstPtr& execute_msg){
        //Generate a plan...            
        std::vector<double> initial_pose =  execute_msg->start_point.positions;
        double tau =  execute_msg->tau;
        double dt =  execute_msg->dt;
        std::vector<double> goal_pose =  execute_msg->target_point.positions;

        trajectory_msgs::JointTrajectory trajectory;

        getDmpPlan(trajectory, initial_pose,goal_pose,tau,dt);
        
        if(trajectory.points.size()==0)
        {
            ROS_ERROR("DMP returned empty path");
            return;
        }

        trajectory_msgs::JointTrajectory::Ptr trajectory_ptr =  boost::make_shared<trajectory_msgs::JointTrajectory>(trajectory);
        
        std::cout<<"Starting plan..."<<std::endl;
  
        std_msgs::Int32 robot_exectuion_status; //0-stopped 1-running 2-success 3-failed
        robot_exectuion_status.data = 1;
        robot_execution_status_publisher.publish(robot_exectuion_status);

        double execution_time = trajectory.points.back().time_from_start.toSec() -  trajectory.points.front().time_from_start.toSec();
        std::cout<<"Execution time: "<<execution_time<<std::endl;
        double collision_distance = -1;
        std::string collision_object_name = "";
        
        // Get goal state
        robot_state::RobotState goal_robot_state = *move_group_ptr->getCurrentState();
        goal_robot_state.setJointGroupPositions(move_group_ptr->getRobotModel()->getJointModelGroup("manipulator"),
                                            trajectory.points.back().positions);
        robot_state::RobotState current_state = *move_group_ptr->getCurrentState();
        double goal_dist = current_state.distance(goal_robot_state);
    
        // Start movement
        //move_group_ptr->asyncExecute(getPlanFromJointTrajectory(trajectory_ptr));
        moveit_msgs::MoveItErrorCodes error = move_group_ptr->asyncExecute(getPlanFromJointTrajectory(trajectory_ptr));
        std::cout<<"Error: "<<error.val<<std::endl;
        std::cout<<"Moving..."<<std::endl;
        auto execution_start_time = std::chrono::high_resolution_clock::now();
        int current_point = 0;
        
        std::cout<<"Start time"<<trajectory_ptr->points[0].time_from_start<<std::endl;

        while(current_point<trajectory_ptr->points.size()-1 && goal_dist>goal_threshold)
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
            current_point = getCurrentTrajectoryPoint(trajectory_ptr->points);
            collision_distance = getCollisionDistance(trajectory_ptr->points, collision_object_name,current_point);            
            //std::cout<<"Current point: "<<current_point<<"/"<<trajectory_ptr->points.size()<<" - Collision dist: "<<collision_distance<<std::endl;
            if(debug)
                std::cout<<"Current point: "<<current_point<<"/"<<trajectory_ptr->points.size()<<" - Collision dist: "<<collision_distance<<" - Duration: "<<execution_duration<<" -  Goal dist: "<<goal_dist<<std::endl;
            if(collision_distance>0 && collision_distance<collision_threshold)
            {   
                std::cout<<"Current point: "<<current_point<<"/"<<trajectory_ptr->points.size()<<" - Collision dist: "<<collision_distance<<" - Duration: "<<execution_duration<<" -  Goal dist: "<<goal_dist<<std::endl;
                std::cout<<"Stoped"<<std::endl;
                move_group_ptr->stop(); // has some delay btween sent and stoped
                auto stop_start_time = std::chrono::high_resolution_clock::now();
                robot_exectuion_status.data = 0;
                robot_execution_status_publisher.publish(robot_exectuion_status);
                sleep(5);
                current_point = getCurrentTrajectoryPoint(trajectory_ptr->points);
                collision_distance = getCollisionDistance(trajectory_ptr->points, collision_object_name,current_point);            
                //std::cout<<"Current point: "<<current_point<<"/"<<trajectory_ptr->points.size()<<" - Collision dist: "<<collision_distance<<std::endl;            
                /*
                //Wait for object to clear path
                while(collision_distance>0 && collision_distance<collision_threshold)
                {
                    current_point = getCurrentTrajectoryPoint(trajectory_ptr->points);
                    collision_distance = getCollisionDistance(trajectory_ptr->points, collision_object_name, current_point);
                    std::cout<<"Current point: "<<current_point<<" - Collision dist: "<<collision_distance<<std::endl;
                }
                */
                std::cout<<"Getting cartesian DMP..."<<std::endl;
                std::vector<geometry_msgs::Pose> path;
                getCartesianDmpPlan(trajectory_ptr, current_point, collision_object_name, cartesian_dmp_n_weights_per_dim, cartesain_dmp_gamma, cartesain_dmp_beta, cartesian_dmp_dt, cartesain_dmp_k, cartesain_dmp_d);
                //trajectory_ptr =  boost::make_shared<trajectory_msgs::JointTrajectory>(trajectory);
                std::cout<<"Start time"<<trajectory_ptr->points[0].time_from_start<<std::endl;

                // TODO add custom validity check
                sleep(60);
                current_point = 0;
                execution_time = trajectory_ptr->points.back().time_from_start.toSec() -  trajectory_ptr->points.front().time_from_start.toSec();
                goal_robot_state.setJointGroupPositions(move_group_ptr->getRobotModel()->getJointModelGroup("manipulator"),
                                            trajectory_ptr->points.back().positions);
                std::cout<<"New execution time: "<<execution_time<<std::endl;
                std::cout<<"Moving..."<<std::endl;
                // move_group_ptr->asyncExecute(getPlanFromJointTrajectory(trajectory_ptr));
                moveit_msgs::MoveItErrorCodes error = move_group_ptr->asyncExecute(getPlanFromJointTrajectory(trajectory_ptr));
                std::cout<<"Error: "<<error.val<<std::endl;
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
    }

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
        req.obstacle=std::vector<double>{};
        req.gamma=std::vector<double>{};
        req.beta=std::vector<double>{};
        req.k=std::vector<double>{};
        std::cout<<"dmp request"<<std::endl;
		dmp::GetDMPPlan::Response res; 
        bool plan_received = get_dmp_plan_client.call(req, res);
        std::cout<<"dmp request done: "<<plan_received<<std::endl;
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

    void getCartesianDmpPlan(trajectory_msgs::JointTrajectory::Ptr& trajectory_ptr, int current_point, std::string collision_object_name,
                             int n_weights_per_dim = 100, double gamma=1000.0, double beta=4.0/M_PI, double k=1000.0,double dt=0.008, double K_gain=100.0, 
                             double D_gain=2.0 * pow(100,0.5), double tau=5, double t_0 = 0, std::vector<double> initial_velocities = {}, 
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
        for (const auto& point : mesh.vertices) {
            obstacle.push_back(point.x);
            obstacle.push_back(point.y);
            obstacle.push_back(point.z);

            obstacleOutputFile << point.x<<" "<<point.y<<" "<<point.z<<'\n';
        }

        // Close the file
        obstacleOutputFile.close();

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
        g_req.beta=std::vector<double>{beta,beta,beta};
        g_req.gamma=std::vector<double>{gamma,gamma,gamma};
        g_req.k=std::vector<double>{k,k,k};
        g_req.obstacle=obstacle;
        dmp::GetDMPPlan::Response g_res; 
        
        bool plan_received = get_cartesian_dmp_plan_client.call(g_req, g_res);

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
        trajectory_msgs::JointTrajectory trajectory = cartesianPathToJointTrajectory(positions, velocities, dt, "eigen");
        
        trajectory_ptr = boost::make_shared<trajectory_msgs::JointTrajectory>(trajectory);
        //sleep(60);
        
        stamped_positions.clear();
        Eigen::Vector3d prev_end_effector_pose;
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
            stamped_positions.push_back(stamped_pose);

            if(i>0)
            {
                double distance = (end_effector_pose.translation() - prev_end_effector_pose).norm();
                //std::cout<<"Point "<<i<<" distance: "<<distance<<std::endl;
            }
            prev_end_effector_pose = end_effector_pose.translation();

        }
        imitated_path.poses = stamped_positions;
        cartesian_ik_path_publisher.publish(imitated_path);

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
        ROS_INFO("Trajectory received");
        trajectory_msgs::JointTrajectoryPoint target_point = trajectory_msg->points.back();
        //bool results = collision_check(trajectory_msg); //Whole path collsion check
    };

private:
    // Ros publishers/subscribers
    ros::Subscriber trajectory_subscriber;
    ros::Subscriber execute_subscriber;
    ros::Publisher robot_execution_status_publisher;
    ros::Publisher cartesian_dmp_path_publisher;
    ros::Publisher cartesian_ik_path_publisher;

    
    planning_scene::PlanningScene *planning_scene_ptr;
    moveit::planning_interface::MoveGroupInterface *move_group_ptr; 

    bool debug;

    double collision_threshold;
    int goal_threshold;

    std::string end_effector_link;
    std::string end_effector_collision_link;

    ros::ServiceClient get_dmp_plan_client;
    ros::ServiceClient get_cartesian_dmp_plan_client;
    ros::ServiceClient learn_dmp_from_demo_client;
    ros::ServiceClient set_active_dmp_client;
    ros::ServiceClient scene_client;

    // Cartesian DMP
    double cartesian_dmp_speed_scale;
    double cartesian_dmp_dt;
    double cartesain_dmp_gamma;
    double cartesain_dmp_beta;
    double cartesain_dmp_k;
    double cartesain_dmp_d;
    int cartesian_dmp_n_weights_per_dim;
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
