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


def get_cube(o_x, o_y, o_z, W=0.1, D=0.1, H=0.1):
    phi = np.arange(1,10,2)*np.pi/4
    Phi, Theta = np.meshgrid(phi, phi) 

    x = np.cos(Phi)*np.sin(Theta)
    y = np.sin(Phi)*np.sin(Theta)
    z = np.cos(Theta)/np.sqrt(2)

    # Change the centroid of the cube from zero to values in data frame
    x = x*W + o_x
    y = y*D + o_y
    z = z*H + o_z

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

    plot_obstacle_bbox = True
    #obstacle_bbox = [0.05,0.05,0.05]  # centroid X,Y,Z and size W,D,H
    obstacle_bbox = [0.1, 0.15, 0.2]

    #obstacle_centroid = np.array([-0.157398, 0.376722, 0.20578])
    obstacle_centroid = np.array([-0.2, 0.4, 0.1])
    
    # Path from file
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

    ax = plt.figure(figsize=(15,12)).add_subplot(projection='3d')
    linewidth = 2.0
    radius = 50
    fontsize = 20
    labelpad=20

    # Assigm real obtacle to plot
    ax.scatter(obstacle_centroid[0], obstacle_centroid[1],obstacle_centroid[2], s=radius, color='tab:orange', label="Obstacle centroid")
    if plot_obstacle_bbox:
        xs,ys,zs = get_cube(obstacle_centroid[0],obstacle_centroid[1],obstacle_centroid[2],obstacle_bbox[0],obstacle_bbox[1],obstacle_bbox[2])
        ax.plot_surface(xs, ys, zs, color='tab:blue',alpha=0.3)
    ax.plot(traj[:, 0], traj[:, 1], traj[:, 2], 'tab:red', label="Demo",linewidth=linewidth)
    ax.scatter([[traj[0, 0], traj[-1, 0]]], [[traj[0, 1], traj[-1, 1]]], [[traj[0, 2], traj[-1, 2]]], s=radius, color='tab:red')
    ax.plot(dP[:, 0],dP[:, 1],dP[:, 2], 'tab:green', label="Desired", linewidth=linewidth)
    ax.scatter([[dP[0, 0], dP[-1, 0]]],[[dP[0, 1], dP[-1, 1]]],[[dP[0, 2], dP[-1, 2]]], s=radius, color='tab:green')
    ax.set_xlim([0.6,-0.6])
    ax.set_ylim([1.1,-0.3])
    ax.set_zlim([-0.6,0.6])
    ax.tick_params(labelsize=fontsize)
    ax.set_xlabel('x', fontsize=fontsize*1.5, labelpad=labelpad)
    ax.set_ylabel('y', fontsize=fontsize*1.5, labelpad=labelpad)
    ax.set_zlabel('z', fontsize=fontsize*1.5, labelpad=labelpad)
    #ax.invert_yaxis() # Match robot base axis
    ax.legend(fontsize=fontsize, ncol=3, loc="lower center", bbox_to_anchor=(0.5, -0.05))
    ax.set_title("DMP with APF", fontsize=fontsize*2, y=1.05)
    plt.tight_layout()
    plt.show()
    