#!/usr/bin/env python3

import rospy
import os
import numpy as np
from tf.transformations import euler_from_quaternion
from dmp.srv import GetDMPPlan,LearnDMPFromDemo, SetActiveDMP
from dmp.msg import DMPTraj, DMPPoint
import time
import math
from scipy.interpolate import CubicSpline
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
            
    def getPlan(self, initial_pose, goal_pose, seg_length=-1, initial_velocities=[], t_0 = None, tau=5, dt=0.008, integrate_iter=1,goal_thresh=[], obstacle=[], beta=[1000.0,0,0], gamma=[20.0 / math.pi,0,0], k=[3.0 / math.pi,0,0],scale_m=0.0, scale_n=1.0):
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
                               seg_length, this_tau, this_dt, this_integrate_iter, obstacle, beta, gamma, k,scale_m, scale_n)
        return plan_resp

    def makePlanRequest(self, x_0, x_dot_0, t_0, goal, goal_thresh,
                        seg_length, tau, dt, integrate_iter, obstacle, beta, gamma,k,scale_m, scale_n):
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
                       seg_length, tau, dt, integrate_iter, obstacle, beta, gamma,k,scale_m, scale_n)
        except rospy.ServiceException as e:
            rospy.logerr("Service call fails: %s"%e)
            exit()
        fin_time = time.time()
        rospy.loginfo("DMP planning done, took: " + str(fin_time - init_time))
    
        return resp

def generate_cube_mesh(center, size, resolution):
    if len(size)<3:
        size=[size,size,size]
     
    if resolution=='auto':
        resolution=[int(size[0]/0.02), int(size[1]/0.02), int(size[2]/0.02)]
    elif len(resolution)<3:
        resolution=[resolution,resolution,resolution]

    # Generate meshgrid
    x = np.linspace(center[0] - size[0]/2, center[0] + size[0]/2, resolution[0])
    y = np.linspace(center[1] - size[1]/2, center[1] + size[1]/2, resolution[1])
    z = np.linspace(center[2] - size[2]/2, center[2] + size[2]/2, resolution[2])

    x, y, z = np.meshgrid(x, y, z)

    # Flatten the meshgrid and organize into a numpy array
    vertices = np.column_stack((x.flatten(), y.flatten(), z.flatten()))

    return vertices

def get_cube(center, size):

    if len(size)<3:
        size=[size,size,size]

    phi = np.arange(1,10,2)*np.pi/4
    Phi, Theta = np.meshgrid(phi, phi) 

    x = np.cos(Phi)*np.sin(Theta)
    y = np.sin(Phi)*np.sin(Theta)
    z = np.cos(Theta)/np.sqrt(2)

    # Change the centroid of the cube from zero to values in data frame
    x = x*size[0] + center[0]
    y = y*size[1] + center[1]
    z = z*size[2] + center[2]

    return x,y,z    
    
def calculate_bounding_box_dimensions(points):
    # Convert points to a numpy array for easier manipulation
    points_array = np.array(points)

    # Calculate min and max values for each dimension
    min_values = np.min(points_array, axis=0)
    max_values = np.max(points_array, axis=0)

    # Calculate dimensions
    dimensions = max_values - min_values

    return dimensions

if __name__ == "__main__":

    rospy.init_node("test_generation_classes")
    rospy.loginfo("Initializing dmp_generation test.")

    mg = motionGeneration()

    dt=0.01
    K=100
    D=2.0 * np.sqrt(K)
    num_bases=50

    gamma=[200.0,300.0,30.0]
    beta=[4.0,2.5,0]
    k=[2.0,5.0,20.0]

    scale_m = 2.0 
    scale_n = 0.4

    dims = 6

    # Cube mesh params
    #obstacle_centroid_list = [[-0.1, 0.4, 0.25]]
    #obstacle_centroid_list = [[-0.1, 0.45, 0.35]]
    #obstacle_size_list = [[0.1,0.1,0.1],[0.2,0.2,0.2],[0.3,0.3,0.3]]

    #obstacle_centroid_list = [[-0.35, 0.3, 0.2],[0.35, 0.3, 0.2],[0.45, 0.3, 0.15]]
    #obstacle_size_list = [[0.15,0.15,0.15]]

    obstacle_centroid_list = [[0.0, 0.45, 0.35]]
    obstacle_size_list = [[0.1,0.1,0.1],[0.2,0.2,0.2],[0.3,0.3,0.3]]

    obstacle_mesh_resolution = 'auto'

    wp_x = [-0.5,-0.1,0.0,0.1,0.5]
    wp_y = [0.3,0.4,0.4,0.4,0.3]
    wp_z = [0.1,0.3,0.35,0.3,0.1]

    wp_x = [-0.5,-0.25,0.0,0.25,0.5]
    wp_y = [0.4,0.4,0.4,0.3,0.2]
    wp_z = [0.5,0.4,0.4,0.2,0.1]


    # Parametric path
    samples = 1000
    traj = np.zeros((samples, 6))
    
    t=np.linspace(0, 10, len(wp_x))
    cs_x = CubicSpline(t, wp_x)
    cs_y = CubicSpline(t, wp_y)
    cs_z = CubicSpline(t, wp_z)
    t_fine = np.linspace(0, 10, samples)
    traj[:, 0] = cs_x(t_fine)
    traj[:, 1] = cs_y(t_fine)
    traj[:, 2] = cs_z(t_fine)

    traj[:, 3] = 1.2
    traj[:, 4] = 0.4
    traj[:, 5] = -0.8

    mg.motion_x0 = traj[0,:].tolist()
    mg.motion_goal = traj[-1,:].tolist()

    resp = mg.makeLFDRequest(dims,traj.tolist(),dt,K,D,num_bases)
    # Set it as the active DMP on the DMP server( ros_DMP)
    mg.makeSetActiveRequest(resp.dmp_list)
    mg.resp_from_makeLFDRequest = resp
    
    initial_pose = traj[0,:].tolist()
    final_pose = traj[-1,:].tolist()

    views = ['auto','xz','xy'] # ['auto'] ['xz','xy','yz']

    fig = plt.figure(figsize=(20,4.5*len(views)))

    if len(obstacle_centroid_list)<len(obstacle_size_list):
        obstacle_centroid_list=[obstacle_centroid_list[0],obstacle_centroid_list[0],obstacle_centroid_list[0]]

    if len(obstacle_centroid_list)>len(obstacle_size_list):
        obstacle_size_list=[obstacle_size_list[0],obstacle_size_list[0],obstacle_size_list[0]]

    for i,(obstacle_size,obstacle_centroid) in enumerate(zip(obstacle_size_list,obstacle_centroid_list)):
        for v,view in enumerate(views):
            # Generate cube mesh
            obstacle_vertices = generate_cube_mesh(obstacle_centroid, obstacle_size, obstacle_mesh_resolution)
            
            bounding_box_dimensions = calculate_bounding_box_dimensions(obstacle_vertices)
            print("Bounding Box Dimensions (X, Y, Z):", bounding_box_dimensions)

            
            obstacle=obstacle_vertices.reshape(-1)

            pla = DMPTraj()  
            pla = mg.getPlan(initial_pose,final_pose,-1,[],None,tau=5,dt=0.008, obstacle=obstacle, beta=beta, gamma=gamma, k=k, scale_m=scale_m, scale_n=scale_n)
            #pla = mg.getPlan(initial_pose,final_pose,-1,[],None,tau=5,dt=0.008, obstacle=obstacle_centroid, beta=beta, gamma=gamma, k=k)

            velocities=[]  
            positions=[]
            for n in range(len(pla.plan.times)):
                positions.append(pla.plan.points[n].positions)
                velocities.append(pla.plan.points[n].velocities)

            dP = np.asarray(positions)
            dV = np.asarray(velocities)

            
            ax = plt.subplot(len(views), len(obstacle_centroid_list), (v*len(obstacle_centroid_list))+i+1, projection='3d')
           
            linewidth = 2.0
            radius = 50
            fontsize = 15
            labelpad = 8
            lims = [[0.6,-0.6],[1.1,-0.3],[-0.4,0.8]]

            auto_dist = 4.5
            auto_axis_off = True
            
            ax.plot(traj[:, 0], traj[:, 1], traj[:, 2], 'tab:red', label="Demo",linewidth=linewidth)
            ax.scatter([[traj[0, 0], traj[-1, 0]]], [[traj[0, 1], traj[-1, 1]]], [[traj[0, 2], traj[-1, 2]]], s=radius, color='tab:red')
            ax.plot(dP[:, 0],dP[:, 1],dP[:, 2], 'tab:green', label="Desired", linewidth=linewidth)
            ax.scatter([[dP[0, 0], dP[-1, 0]]],[[dP[0, 1], dP[-1, 1]]],[[dP[0, 2], dP[-1, 2]]], s=radius, color='tab:green')
            
            ax.scatter(obstacle_centroid[0], obstacle_centroid[1],obstacle_centroid[2], s=radius, color='tab:orange', label="Obstacle centroid")
            x,y,z = get_cube(obstacle_centroid,obstacle_size)
            surf = ax.plot_surface(x,y,z, alpha=0.3, color='tab:blue')
            ax.plot([], [], 's', alpha=0.3, color='tab:blue', label="Obstacle")
            #ax.scatter(obstacle_vertices[:,0], obstacle_vertices[:,1],obstacle_vertices[:,2], s=1, color='tab:blue', label="Obstacle",alpha=0.2)
            
            ax.xaxis.pane.fill = False
            ax.yaxis.pane.fill = False
            ax.zaxis.pane.fill = False

            ax.set_xlim(lims[0])
            ax.set_ylim(lims[1])
            ax.set_zlim(lims[2])

            if view=='auto':
                ax.tick_params(labelsize=fontsize*0.6)
                ax.set_xlabel('X(m)', fontsize=fontsize*1, labelpad=labelpad*2)
                ax.set_ylabel('Y(m)', fontsize=fontsize*1, labelpad=labelpad*2)
                ax.set_zlabel('Z(m)', fontsize=fontsize*1, labelpad=labelpad*2)
                if auto_axis_off:
                    ax.set_axis_off()
                    #ax.view_init(azim=140)
                    ax.view_init(azim=-45)
                    ax.dist = auto_dist
                    ax.invert_xaxis()
            else:
                ax.tick_params(labelsize=fontsize)
                ax.set_xlabel('X(m)', fontsize=fontsize*1, labelpad=labelpad*1.5)
                ax.set_ylabel('Y(m)', fontsize=fontsize*1, labelpad=labelpad*1.5)
                ax.set_zlabel('Z(m)', fontsize=fontsize*1, labelpad=labelpad*1.5)

            if v==0:
                if len(obstacle_size_list)>1:
                    title_txt="Obstacle size: {},{},{}".format(obstacle_size[0],obstacle_size[1],obstacle_size[2])
                if len(obstacle_centroid_list)>1:
                    try:
                        title_txt+="\nCentroid: {},{},{}".format(obstacle_centroid[0],obstacle_centroid[1],obstacle_centroid[2])
                    except:
                        title_txt="Centroid: {},{},{}".format(obstacle_centroid[0],obstacle_centroid[1],obstacle_centroid[2])
                ax.set_title(title_txt, fontsize=fontsize*1.5, y=1.05)
                #ax.set_title("Centroid: {},{},{}".format(obstacle_centroid[0],obstacle_centroid[1],obstacle_centroid[2]), fontsize=fontsize*1.5, y=1.05)

            if view=='xz':
                ax.view_init(elev=0, azim=90)
                ax.dist = 6
                ax.set_proj_type('ortho')
                ax.set_yticks([])
                ax.set_ylabel('')
            elif view=='xy':
                ax.view_init(elev=-90, azim=90)
                ax.dist = 6
                ax.set_proj_type('ortho')
                ax.set_zticks([])
                ax.set_zlabel('')
            elif view=='yz':
                ax.view_init(elev=0, azim=0)
                ax.dist = 6
                ax.set_proj_type('ortho')
                ax.set_xticks([])
                ax.set_xlabel('')
        
            if i == 0:
                handles, labels = ax.get_legend_handles_labels()

    fig.suptitle("DMP with APF",fontsize=fontsize*2)
    #ax.legend(fontsize=fontsize*0.75, loc='center left',bbox_to_anchor=(1, 0.5))
    fig.legend(handles, labels, loc='lower center', ncol=4, fontsize="15")
    fig.tight_layout()
    #fig.subplots_adjust(wspace=0.01,hspace=0.001)
    fig.subplots_adjust(left=0.1, right=0.9, bottom=0.1, top=0.9, wspace=0.5, hspace=0.3)
    plt.show()
    