#!/usr/bin/python


from numpy.core.numeric import _full_like_dispatcher
import rospy
from moveit_msgs.msg import RobotState, RobotTrajectory, DisplayRobotState
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from moveit_msgs.msg._DisplayRobotState import DisplayRobotState
from std_msgs.msg import Bool, Int32
import time
from moveit_msgs.srv import  GetPositionIK
from tf.transformations import *
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import sys
import os
sys.path.append(os.getcwd()+"/src/roy_dmp/src")
from kinematics_interface import *
import roslib#; roslib.load_manifest('ur_driver')
import actionlib
from trajectory_msgs.msg import *
from math import pi

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
        self.robot_state_collision_pub = rospy.Publisher('/robot_collision_state', DisplayRobotState,queue_size=1)
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


    def safe_stop_callback(self, msg):
        self.safe_stop=msg.data
        #print('callback:',self.safe_stop)

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

    def checkTrajectoryValidity(self, robot_trajectory, groups=[]):
        """Given a robot trajectory, deduce it's groups and check it's validity on each point of the traj
        returns True if valid, False otherwise
        It's considered not valid if any point is not valid"""

        init_time = time.time()
        if len(groups) > 0:
            groups_to_check = groups
        else:
            groups_to_check = ['manipulator'] # Automagic group deduction... giving a group that includes everything 
        for traj_point in robot_trajectory.joint_trajectory.points:
            rs = RobotState()
            rs.joint_state.name = robot_trajectory.joint_trajectory.joint_names
            rs.joint_state.position = traj_point.positions
            for group in groups_to_check:
                result = self.sv.getStateValidity(rs,group)
                if not result.valid:
                    rospy.logerr("Trajectory is not valid at point (RobotState):" + str(rs) + "with result of StateValidity: " + str(result))
                    rospy.logerr("published in /robot_collision_state the conflicting state")
                    return False
            fin_time = time.time()
        rospy.logwarn("Trajectory validity of " + str(len(robot_trajectory.joint_trajectory.points)) + " points took " + str(fin_time - init_time))
        return True

  
    def sendTrajectoryAction(self,pla,_initial_pose,simulation):
        if simulation:
            print('Simulation')
            client = actionlib.SimpleActionClient('eff_joint_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        else: #scaled_pos_joint_traj_controller
            print('Real\n')
            client = actionlib.SimpleActionClient('scaled_pos_joint_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        
        print('Waiting for server')
        client.wait_for_server()
        print('Server conected')
        g = FollowJointTrajectoryGoal()
        g.trajectory = JointTrajectory()
        g.trajectory.joint_names = self.arm
        initial_pose = _initial_pose
        
        try:
            times = pla.plan.times
            d= 2.00
            g.trajectory.points = [JointTrajectoryPoint(positions=initial_pose, velocities=[0]*6, time_from_start=rospy.Duration(0.0))]
            for i in range(len(pla.plan.times)):
                joint_value = pla.plan.points[i].positions
                velocity = pla.plan.points[i].velocities
                Q = [joint_value[0],joint_value[1],joint_value[2],joint_value[3],joint_value[4],joint_value[5]]
                V = [velocity[0],velocity[1],velocity[2],velocity[3],velocity[4],velocity[5]]
                T = times[i]
                g.trajectory.points.append(JointTrajectoryPoint(positions=Q, velocities=V, time_from_start=rospy.Duration(T)))
            print('Sending goal')
            client.send_goal(g)
            print('Goal sent')
            
            # Check if the action has finished
            #client.wait_for_result(rospy.Duration(0))
            safe_stop = 0
            while not rospy.is_shutdown():
                #print('while:',self.safe_stop)
                if self.safe_stop:
                    client.cancel_goal()
                    
                    print('Goal cancelled')
                    return False
                
                result = client.get_result()
                #res: error_code: -4 error_string: "shoulder_lift_joint path error -0.484287" 
                #<class 'control_msgs.msg._FollowJointTrajectoryResult.FollowJointTrajectoryResult'>
                if result != None: 
                    rospy.loginfo("Goal achieved: "+ error_codes[result.error_code])
                    break  # Exit the loop when the action is completed


        except KeyboardInterrupt:
            client.cancel_goal()
            raise
        except:
            print("Fault")
            raise
        return True

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
        for itr in range(len(_path.plan.points)):
            joint_positions = _path.plan.points[itr].positions
            path = self.fkPath(joint_positions,_linkName)
            pose_stamped = PoseStamped()
            pose_stamped.pose.position.x = path.position.x
            pose_stamped.pose.position.y = path.position.y
            pose_stamped.pose.position.z = path.position.z
            imitated_path.poses.append(pose_stamped)
        self.imitated_path_pub.publish(imitated_path)


if __name__ == "__main__":

    rospy.init_node("test_execution_classes")
    rospy.loginfo("Initializing dmp_execution test.")
    me = motionExecution()
    me.fkPath()
    
          
