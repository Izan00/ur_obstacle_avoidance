#!/usr/bin/env python3

import rospy
from gazebo_msgs.srv import SpawnModel, DeleteModel, SetModelState, GetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose
import numpy as np
import keyboard
import time
import os

def auto_move_model(model):
    # Create a service proxy for setting the model state in Gazebo
    set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

    # Create a ModelState message to specify the new pose of the cube
    model_state = ModelState()
    model_state.model_name = model  # Replace with the name of your cube model
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

def get_model_state(model):
    rospy.wait_for_service('/gazebo/get_model_state')
    try:
        get_model_state_proxy = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        model_state = get_model_state_proxy(model, 'world')
        return model_state
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

def move_model(model):
    set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

    model_state = get_model_state(model)

    if keyboard.is_pressed('up'):
        model_state.pose.position.z+=pose_increment
    if keyboard.is_pressed('down'):
        model_state.pose.position.z-=pose_increment
    if keyboard.is_pressed('w'):
        model_state.pose.position.y+=pose_increment
    if keyboard.is_pressed('s'):
        model_state.pose.position.y-=pose_increment
    if keyboard.is_pressed('a'):
        model_state.pose.position.x+=pose_increment
    if keyboard.is_pressed('d'):
        model_state.pose.position.x-=pose_increment

    # Send the new model state to Gazebo
    try:
        set_model_state(model_state)
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
    
    try:
        time.sleep(time_sleep)
    except:
        pass


def delete_model(model):
    # Create a service proxy for deleting a model in Gazebo
    delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)

    # Call the service to delete the specified model
    try:
        delete_model(model)
        rospy.loginfo(f"Deleted model: {model}")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

def load_model(model):
    # Define the path to your SDF model file (e.g., cube.sdf)
    model_path = os.path.join(os.getcwd(),"src/obstacle_avoidance/models/"+model+".sdf")
    
    # Set the pose of the cube
    cube_pose = Pose()
    cube_pose.position.x = initial_pose[0]
    cube_pose.position.y = initial_pose[1]
    cube_pose.position.z = initial_pose[2]
    
    # Load the model
    print('Wainting gazebo')
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    print('Spawning')
    try:
        spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        spawn_model(model, open(model_path, 'r').read(), "robot_namespace", cube_pose, "world")
        rospy.loginfo(f"Successfully spawned {model} in Gazebo!")
    except rospy.ServiceException as e:
        rospy.logerr(f"Failed to spawn {model} in Gazebo: {e}")


if __name__ == '__main__':
    
    rospy.init_node('load_obstacle')

    model = ['cube', 'arm'][1]    
    
    pose_increment = 0.01 # in m
    time_sleep = 1.0
    # Moving speed pose_increment/sleep

    initial_pose = [0.0,0.5,0.1]

    try:
        delete_model(model)
    except:
        pass

    try:
        load_model(model)
    except:
        pass

    '''
    time.sleep(60)
    try:
        move_cube(model_name)
    except rospy.ROSInterruptException:
        pass
    '''

    while not rospy.is_shutdown():
        move_model(model)