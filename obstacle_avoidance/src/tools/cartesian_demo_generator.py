#!/usr/bin/env python3

import rospy
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander
from geometry_msgs.msg import Pose, Point, Quaternion

if __name__ == "__main__":
    rospy.init_node('moveit_trajectory_example', anonymous=True)

    # Initialize MoveIt!
    robot = RobotCommander()
    group_name = "manipulator"
    move_group = MoveGroupCommander(group_name)
    planning_scene_interface = PlanningSceneInterface()

    rospy.loginfo("Current pose:")
    print(move_group.get_current_pose().pose)
    rospy.loginfo("Current joints:")
    print(move_group.get_current_joint_values())

    eef_step = 0.01
    jump_threshold = 0.0

    # Starting pose in joint space
    start_joints = [-0.22615668257099575, -1.7335379154343649, -1.74481119221611, 5.238985930603836, 1.1925158255026886, 1.2315106659518529]

    # Cartesian path points
    waypoints=[Pose(position=Point(x=-0.3,y=0.21,z=0.25),orientation=Quaternion(x=-0.9772161619776188,y=-0.036744638425824615,z=0.03733917628376333,w=0.20567982456025632)),
               Pose(position=Point(x=0.2,y=0.22,z=0.35),orientation=Quaternion(x=-0.9772161619776188,y=-0.036744638425824615,z=0.03733917628376333,w=0.20567982456025632)),
               Pose(position=Point(x=0.38,y=0.22,z=0.25),orientation=Quaternion(x=-0.9772161619776188,y=-0.036744638425824615,z=0.03733917628376333,w=0.20567982456025632))]

    rospy.loginfo("Moving to start...")
    # Go to start joints pose and wait
    move_group.set_joint_value_target(start_joints)
    move_group.go(wait=True)

    rospy.logwarn("Press Enter to start executing plan...")
    input("")

    (plan, fraction) = move_group.compute_cartesian_path(waypoints,eef_step, 0.0)          
    if fraction<1:
        rospy.logwarn("Path incomplete, fraction: ", fraction)

    rospy.loginfo('Moving...')
    # Execute the trajectory
    move_group.execute(plan, wait=True)

    rospy.loginfo("Moved")
