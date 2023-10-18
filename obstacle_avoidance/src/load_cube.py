#!/usr/bin/env python3

import rospy
from gazebo_msgs.srv import SpawnModel, DeleteModel, SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose
import numpy as np
import time

def move_cube(model_name):
    # Create a service proxy for setting the model state in Gazebo
    set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

    # Create a ModelState message to specify the new pose of the cube
    model_state = ModelState()
    model_state.model_name = model_name  # Replace with the name of your cube model
    print('Moving')
    for i in range(10):
        model_state.pose.position.x = 0.0  
        model_state.pose.position.y = 0.5+0.05*i  
        model_state.pose.position.z = 0.1+0.05*i

        # Send the new model state to Gazebo
        try:
            set_model_state(model_state)
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

        time.sleep(10)
    print('Moved')

def delete_cube(model_name):
    # Create a service proxy for deleting a model in Gazebo
    delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)

    # Call the service to delete the specified model
    try:
        delete_model(model_name)
        rospy.loginfo(f"Deleted model: {model_name}")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

def load_cube(model_name):
    # Define the path to your SDF model file (e.g., cube.sdf)
    model_path = "/home/izan/ur_ws/src/obstacle_avoidance/models/cube.sdf"
    
    # Set the pose of the cube
    cube_pose = Pose()
    cube_pose.position.x = 0.0
    cube_pose.position.y = 0.5
    cube_pose.position.z = 0.1  # Adjust the height as needed
    
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
    
    rospy.init_node('load_cube')

    model_name = 'cube0'

    try:
        delete_cube(model_name)
    except:
        pass

    try:
        load_cube(model_name)
    except:
        pass

    time.sleep(60)
    try:
        move_cube(model_name)
    except rospy.ROSInterruptException:
        pass

    rospy.spin()