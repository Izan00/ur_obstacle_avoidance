#!/usr/bin/python


from numpy.core.numeric import _full_like_dispatcher
import rospy
from moveit_msgs.msg import RobotState, RobotTrajectory, DisplayRobotState
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from moveit_msgs.msg._DisplayRobotState import DisplayRobotState
from std_msgs.msg import Bool, Int32, Header
import time
from moveit_msgs.srv import  GetPositionIK
from tf.transformations import *
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from geometry_msgs.msg import PoseStamped, Pose, Point
from nav_msgs.msg import Path
import sys
import os
sys.path.append(os.getcwd()+"/src/roy_dmp/src")
from kinematics_interface import *
import roslib#; roslib.load_manifest('ur_driver')
import actionlib
from trajectory_msgs.msg import *
from math import pi
import numpy as np
from obstacle_avoidance.msg import AvoidanceExecute

DEFAULT_JOINT_STATES = '/joint_states'
EXECUTE_KNOWN_TRAJ_SRV = '/execute_kinematic_path'
DEFAULT_IK_SERVICE = "/compute_ik"

DEBUG_MODE =  True

error_codes = {0:'SUCCESSFUL', -1:'INVALID_GOAL', -2:'INVALID_JOINTS', -3:'OLD_HEADER_TIMESTAMP', -4:'PATH_TOLERANCE_VIOLATED', -5:'GOAL_TOLERANCE_VIOLATED'}


class motionExecution():

    def __init__(self):
        rospy.loginfo("Initializing motionExecution")
        rospy.loginfo("Connecting to MoveIt! known trajectory executor server '" + EXECUTE_KNOWN_TRAJ_SRV + "'...")
        rospy.loginfo("Connected.")
        self.sv = StateValidity()
        self.fk = ForwardKinematics()
        self.ik = InverseKinematics()
        self.robot_state_collision_pub = rospy.Publisher('/robot_collision_state', DisplayRobotState, queue_size=1)
        #self.robot_trajectory_pub = rospy.Publisher('/robot_global_trajectory', GlobalTrajectory, queue_size=10)
        self.robot_trajectory_pub = rospy.Publisher('/robot_global_trajectory', JointTrajectory, queue_size=1)
        self.execute_avoidance_pub = rospy.Publisher('/avoidance_execute', AvoidanceExecute, queue_size=1)
        rospy.sleep(0.1) # Give time to the publisher to register
        #TODO: make ik_service_name a param to load from a yaml
        self.imitated_path_pub = rospy.Publisher("/imitated_path", Path, queue_size=1)
        self.ik_service_name = DEFAULT_IK_SERVICE
        # Get a ServiceProxy for the IK service
        rospy.loginfo("Waiting for service '" + self.ik_service_name + "'...")
        rospy.wait_for_service(self.ik_service_name)
        self.ik_serv = rospy.ServiceProxy(self.ik_service_name, GetPositionIK)
        rospy.loginfo("Successful connection  to '" + self.ik_service_name + "'.")        
        
        self.arm = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        
        rospy.Subscriber("/robot_safe_stop", Bool,self.safe_stop_callback)
        self.safe_stop = False

        rospy.Subscriber("/robot_execution_status", Int32, self.execution_status_callback)
        self.execution_status = 0 # 0-stopped 1-running 2-success 3-failed

    def safe_stop_callback(self, msg):
        self.safe_stop=msg.data
        #print('callback:',self.safe_stop)

    def execution_status_callback(self, msg):
        self.execution_status=msg.data #0-None 1-stopped 2-running 3-success 4-failed

    def robotTrajectoryFromPlan(self, plan, joint_names):
        """Given a dmp plan (GetDMPPlanResponse) create a RobotTrajectory to be able to visualize what it consists and also
        to be able to send it to execution"""
        rt = RobotTrajectory()
        rt.joint_trajectory.joint_names = joint_names
        for point, time in zip(plan.plan.points, plan.plan.times):
            jtp = JointTrajectoryPoint()
            jtp.positions = point.positions
            jtp.velocities = point.velocities
            jtp.time_from_start = rospy.Duration(time)
            rt.joint_trajectory.points.append(jtp)
        return rt

    def checkTrajectoryValidity(self, robot_trajectory, groups=[], avoidance=False):
        """Given a robot trajectory, deduce it's groups and check it's validity on each point of the traj
        returns True if valid, False otherwise
        It's considered not valid if any point is not valid"""

        init_time = time.time()
        if len(groups) > 0:
            groups_to_check = groups
        else:
            groups_to_check = ['manipulator'] # Automagic group deduction... giving a group that includes everything 
        results = [self.sv.getStateValidity(RobotState(joint_state=JointState(name=robot_trajectory.joint_trajectory.joint_names, position=traj_point.positions)),group).valid for traj_point in robot_trajectory.joint_trajectory.points for group in groups_to_check]
        fin_time = time.time()
        rospy.logwarn("Trajectory validity of " + str(len(robot_trajectory.joint_trajectory.points)) + " points took " + str(fin_time - init_time))
        
        for conflict_id in np.where(np.array(results)==False)[0]:
            traj_point = robot_trajectory.joint_trajectory.points[conflict_id//len(groups_to_check)]
            group = groups_to_check[conflict_id%len(groups_to_check)]
            rs = RobotState(joint_state=JointState(name=robot_trajectory.joint_trajectory.joint_names, position=traj_point.positions))
            conflict = self.sv.getStateValidity(rs, group)
            if not avoidance:
                rospy.logerr("Trajectory is not valid at point (RobotState):" + str(rs) + "with result of StateValidity: " + str(conflict))
                rospy.logerr("published in /robot_collision_state the conflicting state")
                return False
            else:
                for contacts in conflict.contacts:
                    if not('collision' in contacts.contact_body_1 or 'collision' in contacts.contact_body_2):
                        return False
        return True

   
    def sendTrajectoryToAvoidance(self,robot_trajectory,initial_pose):
        trajectory_msg = JointTrajectory()
        #start_point = JointTrajectoryPoint(positions=initial_pose, velocities=[0]*len(self.arm), time_from_start=rospy.Duration(0.001))
        #target_point = JointTrajectoryPoint(positions=target_pose, velocities=[0]*len(self.arm), time_from_start=rospy.Duration(robot_trajectory.plan.times[-1]))
        trajectory = [JointTrajectoryPoint(positions=point.positions, velocities=point.velocities, time_from_start=rospy.Duration(time)) for point,time in zip(robot_trajectory.plan.points,robot_trajectory.plan.times)]
        #trajectory = [start_point] + trajectory #+ [target_point]
        trajectory_msg.header.stamp = rospy.Time.now()
        rospy.loginfo('Sending trajectory...')
        trajectory_msg.joint_names = self.arm
        trajectory_msg.points = trajectory
        self.robot_trajectory_pub.publish(trajectory_msg)
        rospy.loginfo('Trajectory sent')

    def sendDmpToAvoidance(self, initial_pose, target_pose, tau, dt):
        start_point = JointTrajectoryPoint(positions=initial_pose, velocities=[0]*len(self.arm), time_from_start=rospy.Duration(0.0))
        goal_point = JointTrajectoryPoint(positions=target_pose, velocities=[0]*len(self.arm), time_from_start=rospy.Duration(0.0))
        execute_msg = AvoidanceExecute()
        execute_msg.header.stamp = rospy.Time.now()
        execute_msg.start_point = start_point
        execute_msg.target_point = goal_point
        execute_msg.tau=tau
        execute_msg.dt=dt
        self.execute_avoidance_pub.publish(execute_msg)

    def recieveExecutionStatus(self):
        return self.execution_status

    def sendTrajectoryAction(self,pla,_initial_pose,simulation, safe_stop_enabled):
        if simulation:
            client = actionlib.SimpleActionClient('eff_joint_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        else: #scaled_pos_joint_traj_controller
            client = actionlib.SimpleActionClient('scaled_pos_joint_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        rospy.loginfo('Waiting for trajectory action server...')
        client.wait_for_server()
        rospy.loginfo('Trajectory action server connected')
        g = FollowJointTrajectoryGoal()
        g.trajectory = JointTrajectory()
        g.trajectory.joint_names = self.arm
        initial_pose = _initial_pose
        try:
            g.trajectory.points = [JointTrajectoryPoint(positions=initial_pose, velocities=[0]*len(self.arm), time_from_start=rospy.Duration(0.0))]
            g.trajectory.points += [JointTrajectoryPoint(positions=point.positions, velocities=point.velocities, time_from_start=rospy.Duration(time)) for point,time in zip(pla.plan.points,pla.plan.times)]
            rospy.loginfo('Sending goal...')
            client.send_goal(g)
            rospy.loginfo('Goal sent')
            # Check if the action has finished
            #client.wait_for_result(rospy.Duration(0))
            while not rospy.is_shutdown():
                if self.safe_stop and safe_stop_enabled:
                    client.cancel_goal()
                    rospy.logwarn('Robot stopped by safe stop')
                    return False
                result = client.get_result()
                if result != None: 
                    rospy.loginfo("Goal achieved: "+ error_codes[result.error_code])
                    return True # Exit the loop when the action is completed
        except KeyboardInterrupt:
            client.cancel_goal()
            rospy.logwarn('Robot stopped by user')
            return False
        except:
            rospy.logerr("Robot movement failed")
            raise
        

    def fkPath(self,_position,_linkName):
        fk_link_names = _linkName
        joint_names = self.arm
        position = _position
        fk_result = self.fk.getFK(fk_link_names,joint_names,position)
        if fk_result.error_code.val !=1:
            print('MoveItErrorCode: '+str(fk_result.error_code.val))
        return fk_result.pose_stamped[0].pose

    def get_IK_from_Quart(self,_ps):
        ik_link_name = "rg2_eef_link"
        group_name = "manipulator"
        ps = PoseStamped()
        ps.header.frame_id ="base_link"
        ps.pose = _ps
        ik_result = self.ik.getIK(group_name,ik_link_name,ps)
        return ik_result

    def pathPublish(self,_path,_linkName):
        imitated_path = Path()
        imitated_path.header.frame_id = "base_link"
        #imitated_path.header.stamp = rospy.Time.now()
        imitated_path.poses = [PoseStamped(pose=Pose(position=Point(x=path.position.x,y=path.position.y,z=path.position.z))) for path in [self.fkPath(path_point.positions,_linkName) for path_point in _path.plan.points]] 
        self.imitated_path_pub.publish(imitated_path)


if __name__ == "__main__":

    rospy.init_node("test_execution_classes")
    rospy.loginfo("Initializing dmp_execution test.")
    me = motionExecution()
    me.fkPath()
    
          
