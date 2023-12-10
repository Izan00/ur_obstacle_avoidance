#!/usr/bin/env python3

import rospy
from moveit_msgs.msg import DisplayTrajectory
from moveit_msgs.srv import GetStateValidity
from moveit_msgs.srv import GetStateValidityRequest
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander

def main():
    rospy.init_node('moveit_trajectory_example', anonymous=True)

    # Initialize MoveIt!
    robot = RobotCommander()
    group_name = "manipulator"
    move_group = MoveGroupCommander(group_name)
    planning_scene_interface = PlanningSceneInterface()

    # Set the target joint values
    target_joint_values = [0.8067, 0.4054, -0.824, -0.144, 0.0013, 0.4017]
    move_group.set_joint_value_target(target_joint_values)

    # Plan the trajectory
    print('Planning..')
    my_plan = move_group.plan()

    # Visualize the trajectory (optional)
    display_traj = DisplayTrajectory()
    display_traj.trajectory_start = robot.get_current_state()
    display_traj.trajectory.append(my_plan)

    display_publisher = rospy.Publisher('/move_group/display_planned_path', DisplayTrajectory, queue_size=1, latch=True)
    display_publisher.publish(display_traj)

    print('Moving...')
    # Execute the trajectory
    #move_group.execute(my_plan, wait=True)
    move_group.go(wait=True)

    # Monitor path validity
    state_validity_service = "/check_state_validity"
    rospy.wait_for_service(state_validity_service)
    state_validity_client = rospy.ServiceProxy(state_validity_service, GetStateValidity)
    request = GetStateValidityRequest()
    request.group_name = group_name
    request.robot_state = robot.get_current_state()

    is_path_valid = state_validity_client.call(request).valid
    if is_path_valid:
        rospy.loginfo("Path is valid!")
    else:
        rospy.logerr("Path is not valid!")

if __name__ == "__main__":
    main()
