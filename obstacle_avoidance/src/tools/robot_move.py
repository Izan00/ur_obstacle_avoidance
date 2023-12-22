#!/usr/bin/env python3

import rospy
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander
from geometry_msgs.msg import Pose, Point, Quaternion

def main():
    rospy.init_node('moveit_trajectory_example', anonymous=True)

    # Initialize MoveIt!
    robot = RobotCommander()
    group_name = "manipulator"
    move_group = MoveGroupCommander(group_name)
    planning_scene_interface = PlanningSceneInterface()
    move_group.stop() # Ensure robot is not moving

    # Joint target
    target_joint_values=[-2.426,-2.114,-1.289,5.1767,1.9445,-0.345] # Goal
    target_joint_values=[-0.217,-2.211,-1.252,5.3312,1.0424,1.9250] # Start
  
    move_group.set_joint_value_target(target_joint_values)

    '''
    # Cartesian target
    pose=Pose(position=Point(x=-0.3,y=0.21,z=0.25),orientation=Quaternion(x=-0.9772161619776188,y=-0.036744638425824615,z=0.03733917628376333,w=0.20567982456025632))
    
    move_group.set_pose_target()
    '''

    # Plan the trajectory
    rospy.loginfo('Planning..')
    #my_plan = move_group.plan()

    rospy.loginfo('Moving...')
    # Execute the trajectory
    #move_group.execute(my_plan, wait=True)
    move_group.go(wait=True)


if __name__ == "__main__":
    main()
