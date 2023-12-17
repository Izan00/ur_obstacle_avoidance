#!/usr/bin/env python3

import rospy
import os
import numpy as np
from tf.transformations import euler_from_quaternion
from dmp.srv import GetDMPPlan,LearnDMPFromDemo, SetActiveDMP
from dmp.msg import DMPTraj, DMPPoint
import time
import math

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

DEFAULT_JOINT_STATES = "/joint_states"
DEFAULT_FK_SERVICE = "/compute_fk"
DEFAULT_IK_SERVICE = "/compute_ik"

namespace = 'cartesian/'

class motionGeneration():

    def __init__(self):
        pass

    def makeLFDRequest(self, dims, traj, dt, K_gain, D_gain, num_bases):
        """Learn a DMP from demonstration data """
        demotraj = DMPTraj()

        for i in range(len(traj)):
            pt = DMPPoint()
            pt.positions = traj[i]
            demotraj.points.append(pt)
            demotraj.times.append(dt*i)
        k_gains = [K_gain]*dims
        d_gains = [D_gain]*dims

        init_time = time.time()
        try:
            rospy.wait_for_service(namespace+'learn_dmp_from_demo', timeout=5)
        except rospy.exceptions.ROSException as e:
            rospy.logerr("Timeout exceeded while waiting for service %s"%e)
            print("")
            rospy.logerr("DMP node not found, please run/re-run 'roslaunch dmp dmp_cartesian.launch")
            exit()
        try:
            lfd = rospy.ServiceProxy(namespace+'learn_dmp_from_demo', LearnDMPFromDemo)
            resp = lfd(demotraj,k_gains,d_gains,num_bases)
        except rospy.ServiceException as e:
            rospy.logerr("Service call fails: %s"%e)
            exit()
        fin_time = time.time()
        rospy.loginfo("LfD done, took: " + str(fin_time - init_time))

        return resp

    def makeSetActiveRequest(self, dmp_list):
        """ Set a DMP as active on the DMP server"""
        try:
            rospy.wait_for_service(namespace+'set_active_dmp', timeout=5)
        except rospy.exceptions.ROSException as e:
            rospy.logerr("Timeout exceeded while waiting for service %s"%e)
            print("")
            rospy.logerr("DMP node not found, please run/re-run 'roslaunch dmp dmp_cartesian.launch")
            exit()
        try:
            sad = rospy.ServiceProxy(namespace+'set_active_dmp', SetActiveDMP)
            sad(dmp_list)
        except rospy.ServiceException as e:
            rospy.logerr("Service call fails: %s" %e)
            exit()
            
    def getPlan(self, initial_pose, goal_pose, seg_length=-1, initial_velocities=[], t_0 = None, tau=5, dt=0.008, integrate_iter=1,goal_thresh=[], obstacle=[], beta=1000.0, gamma=20.0 / math.pi):
        """Generate a plan..."""

        x_0 = initial_pose
        x_dot_0 = [0.0] * len(initial_pose)
        t_0 = 0
        this_dt= dt
        this_tau = tau*2
        this_integrate_iter = integrate_iter
        goal = goal_pose
        if len(goal_thresh) > 0:
            this_goal_thresh = goal_thresh
        else:
            this_goal_thresh = [0.01] * len(initial_pose)
        seg_length = seg_length          #Plan until convergence to goal is -1

        rospy.logwarn("tau is: " + str(this_tau))
        plan_resp = self.makePlanRequest(x_0, x_dot_0, t_0, goal, this_goal_thresh,
                               seg_length, this_tau, this_dt, this_integrate_iter, obstacle, beta, gamma)
        return plan_resp

    def makePlanRequest(self, x_0, x_dot_0, t_0, goal, goal_thresh,
                        seg_length, tau, dt, integrate_iter, obstacle, beta, gamma):
        """Generate a plan from a DMP """
        print("Starting DMP planning...")
        init_time = time.time()
        try:
            rospy.wait_for_service(namespace+'get_dmp_plan', timeout=5)
        except rospy.exceptions.ROSException as e:
            rospy.logerr("Timeout exceeded while waiting for service %s"%e)
            print("")
            rospy.logerr("DMP node not found, please run/re-run 'roslaunch dmp dmp_cartesian.launch")
            exit()
        try:
            gdp = rospy.ServiceProxy(namespace+'get_dmp_plan', GetDMPPlan)
            resp = gdp(x_0, x_dot_0, t_0, goal, goal_thresh,
                       seg_length, tau, dt, integrate_iter, obstacle, beta, gamma)
        except rospy.ServiceException as e:
            rospy.logerr("Service call fails: %s"%e)
            exit()
        fin_time = time.time()
        rospy.loginfo("DMP planning done, took: " + str(fin_time - init_time))
    
        return resp


def get_cube(o_x, o_y, o_z, L=0.1):   
    phi = np.arange(1,10,2)*np.pi/4
    Phi, Theta = np.meshgrid(phi, phi) 

    x = np.cos(Phi)*np.sin(Theta)
    y = np.sin(Phi)*np.sin(Theta)
    z = np.cos(Theta)/np.sqrt(2)

    # Change the centroid of the cube from zero to values in data frame
    x = x*L + o_x
    y = y*L + o_y
    z = z*L + o_z

    return x,y,z    
    

if __name__ == "__main__":

    rospy.init_node("test_generation_classes")
    rospy.loginfo("Initializing dmp_generation test.")

    mg = motionGeneration()

    dt=0.01
    K=50
    D=2.0 * np.sqrt(K)
    num_bases=100

    gamma=1000.0
    beta=18.0 / math.pi
    
    dims = 6

    plot_obstacle_as_cube = True
    cube_size = 0.05

    # Path from file
    real_obstacle_centorid = np.array([-0.15, 0.37, 0.2])
    obstacle_centroid = np.array([-0.157398, 0.376722, 0.20578])
    
    traj=[]
    file_path = os.path.join(os.getcwd(),'src/obstacle_avoidance/data/2_test_avoidance.txt')
    with open(file_path, 'r') as inputFile:
        lines = inputFile.readlines()
        for line in lines:
            coordinates = [float(coord) for coord in line.strip().split()]
            traj.append(coordinates)
    traj=np.array(traj)

    '''
    # Parametric path
    obstacle_centroid = np.array([0.0, 0.4, 0.1])

    samples = 1000
    traj = np.zeros((samples, 6))
    
    traj[:, 0] = np.linspace(0.35, -0.35, samples)
    traj[:, 1] = np.concatenate((np.linspace(0.3, 0.4, samples//2),np.linspace(0.4, 0.3, samples//2)))
    traj[:, 2] = 0.15

    traj[:, 3] = 1.2
    traj[:, 4] = 0.4
    traj[:, 5] = -0.8
    '''

    mg.motion_x0 = traj[0,:].tolist()
    mg.motion_goal = traj[-1,:].tolist()

    resp = mg.makeLFDRequest(dims,traj.tolist(),dt,K,D,num_bases)
    # Set it as the active DMP on the DMP server( ros_DMP)
    mg.makeSetActiveRequest(resp.dmp_list)
    mg.resp_from_makeLFDRequest = resp
    
    initial_pose = traj[0,:].tolist()
    final_pose = traj[-1,:].tolist()

    pla = DMPTraj()  
    pla = mg.getPlan(initial_pose,final_pose,-1,[],None,tau=5,dt=0.008, obstacle=obstacle_centroid, beta=beta, gamma=gamma)

    velocities=[]  
    positions=[]
    for i in range(len(pla.plan.times)):
        positions.append(pla.plan.points[i].positions)
        velocities.append(pla.plan.points[i].velocities)

    dP = np.asarray(positions)
    dV = np.asarray(velocities)

    ax = plt.figure().add_subplot(projection='3d')
    lims=[-0.6,0.6]

    ax.scatter(obstacle_centroid[0], obstacle_centroid[1],obstacle_centroid[2], s=50, color='tab:orange', label="Obstacle centroid")
    if plot_obstacle_as_cube:
        xs,ys,zs = get_cube(real_obstacle_centorid[0],real_obstacle_centorid[1],real_obstacle_centorid[2],cube_size)
        ax.plot_surface(xs, ys, zs, color='tab:blue',alpha=0.5)
    ax.plot(traj[:, 0], traj[:, 1], traj[:, 2], 'tab:red', label="Demo")
    ax.scatter([[traj[0, 0], traj[-1, 0]]], [[traj[0, 1], traj[-1, 1]]], [[traj[0, 2], traj[-1, 2]]], s=20, color='tab:red')
    ax.plot(dP[:, 0],dP[:, 1],dP[:, 2], 'tab:green', label="Desired")
    ax.scatter([[dP[0, 0], dP[-1, 0]]],[[dP[0, 1], dP[-1, 1]]],[[dP[0, 2], dP[-1, 2]]], s=20, color='tab:green')
    ax.set_xlim(lims)
    ax.set_ylim(lims)
    ax.set_zlim(lims)
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    ax.legend()
    plt.show()
    