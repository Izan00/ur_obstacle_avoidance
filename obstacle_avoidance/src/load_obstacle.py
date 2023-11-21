#!/usr/bin/env python3

import rospy
from gazebo_msgs.srv import SpawnModel, DeleteModel, SetModelState, GetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose
import numpy as np
from pynput import keyboard
import time
import os
import subprocess
import sys

def is_terminal_focused():
    try:
        # Run xdotool to get the active window
        result = subprocess.check_output(["xdotool", "getactivewindow", "getwindowname"], universal_newlines=True)
        #print(result)
        # Check if the window name contains "Terminal" (you might need to adjust this based on your terminal)
        return "Obstacle" in result
    except subprocess.CalledProcessError:
        return False
    
def on_press(key):
    if is_terminal_focused():
        pressed_keys[key.char]=1
        print("           \r", end="")

def on_release(key):
    #print(key)
    if is_terminal_focused():
        pressed_keys[key.char]=0
        print("           \r", end="")

def auto_move_model(model):
    # Create a service proxy for setting the model state in Gazebo
    set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

    # Create a ModelState message to specify the new pose of the cube
    model_state = ModelState()
    model_state.model_name = model  # Replace with the name of your cube model
    rospy.loginfo('Moving')
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
    rospy.loginfo('Moved')

def get_model_state(model):
    rospy.wait_for_service('/gazebo/get_model_state')
    try:
        get_model_state_proxy = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        model_state = get_model_state_proxy(model, 'world')
        return model_state.pose
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

def move_model(model):
    model_state = ModelState()
    set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

    model_state_pose = get_model_state(model)

    model_state.model_name = model
    model_state.pose = model_state_pose

    if pressed_keys['i']==1:
        model_state.pose.position.z+=pose_increment
    if pressed_keys['k']==1:
        model_state.pose.position.z-=pose_increment
    if pressed_keys['w']==1:
        model_state.pose.position.y-=pose_increment
    if pressed_keys['s']==1:
        model_state.pose.position.y+=pose_increment
    if pressed_keys['a']==1:
        model_state.pose.position.x+=pose_increment
    if pressed_keys['d']==1:
        model_state.pose.position.x-=pose_increment

    print("           \r", end="")

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
    rospy.loginfo('Waiting gazebo')
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    rospy.loginfo('Spawning')
    try:
        spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        spawn_model(model, open(model_path, 'r').read(), "robot_namespace", cube_pose, "world")
        rospy.loginfo(f"Successfully spawned {model} in Gazebo!")
    except rospy.ServiceException as e:
        rospy.logerr(f"Failed to spawn {model} in Gazebo: {e}")


if __name__ == '__main__':
    # Change terminal name
    sys.stdout.write("\x1b]2;Obstacle\x07")


    rospy.init_node('load_obstacle')

    model = ['cube', 'arm'][0]    
    
    pose_increment = 0.01 # in m
    time_sleep = 0.1
    # Moving speed pose_increment/sleep

    initial_pose = [0.0,0.5,0.1]

    pressed_keys = {'w':0,'s':0,'d':0,'a':0,'i':0,'k':0}

    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

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

    print('\n\n=========================================================================')
    print("      Welcome to the Joystick Control Interface")
    print('=========================================================================')
    print("\n  Use the following keys for joystick actions:\n\n            w                       i\n         a  s  d                    k\n\n  Move along X and Y axes    Move along Z axis\n\n  Press 'ctrl+c' to finish and exit the program.")
    
    while not rospy.is_shutdown():
        move_model(model)