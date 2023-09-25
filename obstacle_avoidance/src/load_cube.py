#!/usr/bin/env python3

import rospy
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose

def load_cube():
    rospy.init_node('load_cube_node')
    
    # Define the path to your SDF model file (e.g., cube.sdf)
    model_path = rospy.get_param("~model_path", "package://obstacle_avoidance/models/cube.sdf")
    
    # Define the name of the model you want to spawn in Gazebo
    model_name = rospy.get_param("~model_name", "my_cube")
    
    # Set the pose of the cube
    cube_pose = Pose()
    cube_pose.position.x = 0.2
    cube_pose.position.y = 0.0
    cube_pose.position.z = 0.2  # Adjust the height as needed
    
    # Load the model
    print('Wainting gazebo')
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    print('Spawning')
    try:
        spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        spawn_model(model_name, open(model_path, 'r').read(), "robot_namespace", cube_pose, "world")
        rospy.loginfo(f"Successfully spawned {model_name} in Gazebo!")
    except rospy.ServiceException as e:
        rospy.logerr(f"Failed to spawn {model_name} in Gazebo: {e}")

if __name__ == '__main__':
    try:
        load_cube()
    except rospy.ROSInterruptException:
        pass
