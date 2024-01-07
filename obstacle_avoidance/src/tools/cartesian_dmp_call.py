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
from matplotlib.widgets import Slider
from matplotlib.gridspec import GridSpec
from scipy.interpolate import CubicSpline

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
    # Generate meshgrid
    x = np.linspace(center[0] - size[0]/2, center[0] + size[0]/2, resolution)
    y = np.linspace(center[1] - size[1]/2, center[1] + size[1]/2, resolution)
    z = np.linspace(center[2] - size[2]/2, center[2] + size[2]/2, resolution)

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

def update_go_slider(val):
    global dt, K, D,num_bases,gamma,beta,k,obstacle_centroid,obstacle_size 
    gamma[0] = val
    update_plot()

def update_bo_slider(val):
    global dt, K, D,num_bases,gamma,beta,k,obstacle_centroid,obstacle_size 
    beta[0] = val
    update_plot()

def update_ko_slider(val):
    global dt, K, D,num_bases,gamma,beta,k,obstacle_centroid,obstacle_size 
    k[0] = val
    update_plot()

def update_gp_slider(val):
    global dt, K, D,num_bases,gamma,beta,k,obstacle_centroid,obstacle_size 
    gamma[1] = val
    update_plot()

def update_bp_slider(val):
    global dt, K, D,num_bases,gamma,beta,k,obstacle_centroid,obstacle_size 
    beta[1] = val
    update_plot()

def update_kp_slider(val):
    global dt, K, D,num_bases,gamma,beta,k,obstacle_centroid,obstacle_size 
    k[1] = val
    update_plot()

def update_gd_slider(val):
    global dt, K, D,num_bases,gamma,beta,k,obstacle_centroid,obstacle_size 
    gamma[2] = val
    update_plot()

def update_kd_slider(val):
    global dt, K, D,num_bases,gamma,beta,k,obstacle_centroid,obstacle_size 
    k[2] =val
    update_plot()

def update_ox_slider(val):
    global dt, K, D,num_bases,gamma,beta,k,obstacle_centroid,obstacle_size 
    obstacle_centroid[0] = val
    update_plot()

def update_oy_slider(val):
    global dt, K, D,num_bases,gamma,beta,k,obstacle_centroid,obstacle_size 
    obstacle_centroid[1] = val
    update_plot()

def update_oz_slider(val):
    global dt, K, D,num_bases,gamma,beta,k,obstacle_centroid,obstacle_size 
    obstacle_centroid[2] =val
    update_plot()

def update_osx_slider(val):
    global dt, K, D,num_bases,gamma,beta,k,obstacle_centroid,obstacle_size
    global dt, K, D,num_bases,gamma,beta,k,obstacle_centroid,obstacle_size 
    obstacle_size[0] =val
    update_plot()

def update_osy_slider(val):
    global dt, K, D,num_bases,gamma,beta,k,obstacle_centroid,obstacle_size 
    obstacle_size[1] =val
    update_plot()

def update_osz_slider(val):
    global dt, K, D,num_bases,gamma,beta,k,obstacle_centroid,obstacle_size 
    obstacle_size[2] =val
    update_plot()

def update_dt_slider(val): 
    global dt, K, D,num_bases,gamma,beta,k,obstacle_centroid,obstacle_size  
    #dt =val
    dt = round(10**val,3)
    dt_slider.valtext.set_text(dt)
    update_plot()

def update_K_slider(val):
    global dt, K, D,num_bases,gamma,beta,k,obstacle_centroid,obstacle_size  
    K =val
    D=2.0 * np.sqrt(K)
    update_plot()

def update_D_slider(val): 
    global dt, K, D,num_bases,gamma,beta,k,obstacle_centroid,obstacle_size  
    D =val
    update_plot()

                        
def update_plot():
    global mg, ax, fig,traj, dt, K, D,num_bases,gamma,beta,k,obstacle_centroid,obstacle_size,obstacle_mesh_resolution
    ax.clear()
    
    # Generate cube mesh
    obstacle_vertices = generate_cube_mesh(obstacle_centroid, obstacle_size, obstacle_mesh_resolution)
    obstacle=obstacle_vertices.reshape(-1)

    mg.motion_x0 = traj[0,:].tolist()
    mg.motion_goal = traj[-1,:].tolist()

    resp = mg.makeLFDRequest(dims,traj.tolist(),dt,K,D,num_bases)
    # Set it as the active DMP on the DMP server( ros_DMP)
    mg.makeSetActiveRequest(resp.dmp_list)
    mg.resp_from_makeLFDRequest = resp
    
    initial_pose = traj[0,:].tolist()
    final_pose = traj[-1,:].tolist()

    pla = DMPTraj()  
    pla = mg.getPlan(initial_pose,final_pose,-1,[],None,tau=5,dt=dt, obstacle=obstacle, beta=beta, gamma=gamma, k=k, scale_m=scale_m, scale_n=scale_n)
    #pla = mg.getPlan(initial_pose,final_pose,-1,[],None,tau=5,dt=0.008, obstacle=obstacle_centroid, beta=beta, gamma=gamma, k=k)

    velocities=[]  
    positions=[]
    for i in range(len(pla.plan.times)):
        positions.append(pla.plan.points[i].positions)
        velocities.append(pla.plan.points[i].velocities)

    dP = np.asarray(positions)
    dV = np.asarray(velocities)

    ax.plot(traj[:, 0], traj[:, 1], traj[:, 2], 'tab:red', label="Demo",linewidth=linewidth)
    ax.scatter([[traj[0, 0], traj[-1, 0]]], [[traj[0, 1], traj[-1, 1]]], [[traj[0, 2], traj[-1, 2]]], s=radius, color='tab:red')
    dmp_line = ax.plot(dP[:, 0],dP[:, 1],dP[:, 2], 'tab:green', label="Desired", linewidth=linewidth)
    ax.scatter([[dP[0, 0], dP[-1, 0]]],[[dP[0, 1], dP[-1, 1]]],[[dP[0, 2], dP[-1, 2]]], s=radius, color='tab:green')
    
    ax.scatter(obstacle_centroid[0], obstacle_centroid[1],obstacle_centroid[2], s=radius, color='tab:orange', label="Obstacle centroid")
    x,y,z = get_cube(obstacle_centroid,obstacle_size)       
    ax.plot_surface(x,y,z, alpha=0.3, color='tab:blue')   
    ax.plot([], [], 's', alpha=0.3, color='tab:blue', label="Obstacle")         
    #ax.scatter(obstacle_vertices[:,0], obstacle_vertices[:,1],obstacle_vertices[:,2], s=radius, color='tab:blue', label="Obstacle",alpha=0.2)

    ax.set_xlim(xlim)
    ax.set_ylim(ylim)
    ax.set_zlim(zlim)
    ax.dist = viw_dist
    ax.tick_params(labelsize=fontsize*0.9)
    ax.set_xlabel('X(m)', fontsize=fontsize, labelpad=labelpad*3)
    ax.set_ylabel('Y(m)', fontsize=fontsize, labelpad=labelpad*3)
    ax.set_zlabel('Z(m)', fontsize=fontsize, labelpad=labelpad*3)
    ax.xaxis.pane.fill = False
    ax.yaxis.pane.fill = False
    ax.zaxis.pane.fill = False
    #ax.invert_yaxis() # Match robot base axis
    ax.legend(fontsize=fontsize*0.9, ncol=4, loc="lower center", bbox_to_anchor=(0.5, -0.05))
    ax.set_title("DMP with APF", fontsize=fontsize*1.5, y=1.0)

    fig.canvas.draw_idle()

if __name__ == "__main__":
    global mg, ax, fig, traj, dt, K, D,num_bases,gamma,beta,k,obstacle_centroid,obstacle_size,obstacle_mesh_resolution
    rospy.init_node("test_generation_classes")
    rospy.loginfo("Initializing dmp_generation test.")

    sliders_enabled = True

    dt=0.01
    K=100
    D=2.0 * np.sqrt(K)
    scale_m = 0.0
    scale_n = 1.0
    num_bases=50

    gamma=[200.0,300.0,30.0]
    beta=[4.0,2.5,0]
    k=[2.0,5.0,20.0]
    
    dims = 6

    # Obsacle Cube mesh params
    obstacle_centroid = [-0.1, 0.4, 0.25] 
    obstacle_size = [0.3, 0.3, 0.3]
    
    obstacle_mesh_resolution = 10

    # Spline demo waypoints
    wp_x = [-0.5,-0.1,0.0,0.1,0.5]
    wp_y = [0.3,0.4,0.4,0.4,0.3]
    wp_z = [0.1,0.3,0.35,0.3,0.1]

    samples = 1000

    # Import from file override params, use "" to generate from parameters
    path_file = '' #'src/obstacle_avoidance/data/real_test_path.txt' #'src/obstacle_avoidance/data/2_test_avoidance_path.txt'
    obstacle_file = '' #'src/obstacle_avoidance/data/2_test_avoidance_obstacle.txt'


    if obstacle_file!="":
        # Obstacle from file
        obstacle_vertices=[]
        obstacle_file_path = os.path.join(os.getcwd(),obstacle_file)
        with open(obstacle_file_path, 'r') as inputFile:
            lines = inputFile.readlines()
            for line in lines:
                point = [float(values) for values in line.strip().split()]
                obstacle_vertices.append(point)
        obstacle_vertices=np.array(obstacle_vertices)
        obstacle_centroid = np.mean(obstacle_vertices, axis=0)
    else:
        # Generate cube mesh
        obstacle_vertices = generate_cube_mesh(obstacle_centroid, obstacle_size, obstacle_mesh_resolution)
    obstacle=obstacle_vertices.reshape(-1)

    if path_file!='':
        # Path from file
        traj=[]
        path_file_path = os.path.join(os.getcwd(),path_file)
        with open(path_file_path, 'r') as inputFile:
            lines = inputFile.readlines()
            for line in lines:
                coordinates = [float(coord) for coord in line.strip().split()]
                traj.append(coordinates)
        traj=np.array(traj)
    else:
        # Parametric path
        traj = np.zeros((samples, 6))

        # Generate splaine from waypoints
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

    mg = motionGeneration()
    
    mg.motion_x0 = traj[0,:].tolist()
    mg.motion_goal = traj[-1,:].tolist()

    resp = mg.makeLFDRequest(dims,traj.tolist(),dt,K,D,num_bases)
    # Set it as the active DMP on the DMP server( ros_DMP)
    mg.makeSetActiveRequest(resp.dmp_list)
    mg.resp_from_makeLFDRequest = resp
    
    initial_pose = traj[0,:].tolist()
    final_pose = traj[-1,:].tolist()

    pla = DMPTraj()  
    pla = mg.getPlan(initial_pose,final_pose,-1,[],None,tau=5,dt=dt, obstacle=obstacle, beta=beta, gamma=gamma, k=k, scale_m=scale_m, scale_n=scale_n)
    #pla = mg.getPlan(initial_pose,final_pose,-1,[],None,tau=5,dt=0.008, obstacle=obstacle_centroid, beta=beta, gamma=gamma, k=k)

    velocities=[]  
    positions=[]
    for i in range(len(pla.plan.times)):
        positions.append(pla.plan.points[i].positions)
        velocities.append(pla.plan.points[i].velocities)

    dP = np.asarray(positions)
    dV = np.asarray(velocities)
    
    linewidth = 2.0
    radius = 50
    fontsize = 20
    labelpad=10
    viw_dist=11
    slider_height=0.02
    xlim = [0.6,-0.6]
    ylim = [1.1,-0.3]
    zlim = [-0.6,0.6]

    if sliders_enabled:
        fontsize=fontsize*0.75
        fig = plt.figure(figsize=(18,8))
        gs = GridSpec(1, 2, width_ratios=[7, 3]) 
        ax = plt.subplot(gs[0], projection='3d')
    else:
        fig = plt.figure(figsize=(15,12))
        ax = fig.add_subplot(projection='3d')
        
    ax.plot(traj[:, 0], traj[:, 1], traj[:, 2], 'tab:red', label="Demo",linewidth=linewidth)
    ax.scatter([[traj[0, 0], traj[-1, 0]]], [[traj[0, 1], traj[-1, 1]]], [[traj[0, 2], traj[-1, 2]]], s=radius, color='tab:red')
    dmp_line = ax.plot(dP[:, 0],dP[:, 1],dP[:, 2], 'tab:green', label="Desired", linewidth=linewidth)
    ax.scatter([[dP[0, 0], dP[-1, 0]]],[[dP[0, 1], dP[-1, 1]]],[[dP[0, 2], dP[-1, 2]]], s=radius, color='tab:green')
    
    ax.scatter(obstacle_centroid[0], obstacle_centroid[1],obstacle_centroid[2], s=radius, color='tab:orange', label="Obstacle centroid")
    x,y,z = get_cube(obstacle_centroid,obstacle_size)       
    ax.plot_surface(x,y,z, alpha=0.3, color='tab:blue')   
    ax.plot([], [], 's', alpha=0.3, color='tab:blue', label="Obstacle")         
    #ax.scatter(obstacle_vertices[:,0], obstacle_vertices[:,1],obstacle_vertices[:,2], s=radius, color='tab:blue', label="Obstacle",alpha=0.2)

    ax.set_xlim(xlim)
    ax.set_ylim(ylim)
    ax.set_zlim(zlim)
    ax.dist = viw_dist
    ax.tick_params(labelsize=fontsize*0.9)
    ax.set_xlabel('X(m)', fontsize=fontsize, labelpad=labelpad*3)
    ax.set_ylabel('Y(m)', fontsize=fontsize, labelpad=labelpad*3)
    ax.set_zlabel('Z(m)', fontsize=fontsize, labelpad=labelpad*3)
    ax.xaxis.pane.fill = False
    ax.yaxis.pane.fill = False
    ax.zaxis.pane.fill = False
    #ax.invert_yaxis() # Match robot base axis
    ax.legend(fontsize=fontsize*0.9, ncol=4, loc="lower center", bbox_to_anchor=(0.5, -0.05))
    ax.set_title("DMP with APF", fontsize=fontsize*1.5, y=1.0)

    if sliders_enabled:
        # Add a slider axes
        ax2 = plt.subplot(gs[1])
        ax2.set_axis_off()

        ax2.text(0.3, 1.0, 'DMP params', fontsize=fontsize, color='k')

        slider_ax_go = plt.axes([0.75, 0.9, 0.2, slider_height])
        go_slider = Slider(slider_ax_go, 'gamma_o', 0, 1000.0, valinit=gamma[0])
        slider_ax_bo = plt.axes([0.75, 0.85, 0.2, slider_height])
        bo_slider = Slider(slider_ax_bo, 'beta_o', 0, 20.0, valinit=beta[0])
        slider_ax_ko = plt.axes([0.75, 0.8, 0.2, slider_height])
        ko_slider = Slider(slider_ax_ko, 'k_o', 0, 20.0, valinit=k[0])

        slider_ax_gp = plt.axes([0.75, 0.75, 0.2, slider_height])
        gp_slider = Slider(slider_ax_gp, 'gamma_p', 0, 1000.0, valinit=gamma[1])
        slider_ax_bp = plt.axes([0.75, 0.7, 0.2, slider_height])
        bp_slider = Slider(slider_ax_bp, 'beta_p', 0, 20.0, valinit=beta[1])
        slider_ax_kp = plt.axes([0.75, 0.65, 0.2, slider_height])
        kp_slider = Slider(slider_ax_kp, 'k_p', 0, 20.0, valinit=k[1])

        slider_ax_gd = plt.axes([0.75, 0.6, 0.2, slider_height])
        gd_slider = Slider(slider_ax_gd, 'gamma_d', 0, 100.0, valinit=gamma[2])
        slider_ax_kd = plt.axes([0.75, 0.55, 0.2, slider_height])
        kd_slider = Slider(slider_ax_kd, 'k_d', 0, 20.0, valinit=k[2])

        slider_ax_dt = plt.axes([0.75, 0.5, 0.2, slider_height])
        dt_slider = Slider(slider_ax_dt, 'dt', -3, 0, valinit=np.log10(dt))
        dt_slider.valtext.set_text(dt)

        slider_ax_K = plt.axes([0.75, 0.45, 0.2, slider_height])
        K_slider = Slider(slider_ax_K, 'K', 0, 500.0, valinit=K)
        #slider_ax_D = plt.axes([0.75, 0.4, 0.2, slider_height])
        #D_slider = Slider(slider_ax_D, 'D', 0, 500.0, valinit=D)

        # Attach the update function to the slider
        go_slider.on_changed(update_go_slider)
        bo_slider.on_changed(update_bo_slider)
        ko_slider.on_changed(update_ko_slider)

        gp_slider.on_changed(update_gp_slider)
        bp_slider.on_changed(update_bp_slider)
        kp_slider.on_changed(update_kp_slider)

        gd_slider.on_changed(update_gd_slider)
        kd_slider.on_changed(update_kd_slider)

        dt_slider.on_changed(update_dt_slider)
        K_slider.on_changed(update_K_slider)
        #D_slider.on_changed(update_D_slider)

        ax2.text(0.25, 0.33, 'Obstacle params', fontsize=fontsize, color='k')
        slider_ax_ox = plt.axes([0.75, 0.3, 0.2, slider_height])
        ox_slider = Slider(slider_ax_ox, 'obstacle_cx', min(xlim), max(xlim), valinit=obstacle_centroid[0])
        slider_ax_oy = plt.axes([0.75, 0.25, 0.2, slider_height],)
        oy_slider = Slider(slider_ax_oy, 'obstacle_cy', min(ylim), max(ylim), valinit=obstacle_centroid[1])
        slider_ax_oz = plt.axes([0.75, 0.2, 0.2, slider_height])
        oz_slider = Slider(slider_ax_oz, 'obstacle_cz', min(zlim), max(zlim), valinit=obstacle_centroid[2])

        slider_ax_osx = plt.axes([0.75, 0.15, 0.2, slider_height])
        osx_slider = Slider(slider_ax_osx, 'obstacle_sx', 0.01, 1, valinit=obstacle_size[0])
        slider_ax_osy = plt.axes([0.75, 0.1, 0.2, slider_height],)
        osy_slider = Slider(slider_ax_osy, 'obstacle_sy', 0.01, 1, valinit=obstacle_size[1])
        slider_ax_osz = plt.axes([0.75, 0.05, 0.2, slider_height])
        osz_slider = Slider(slider_ax_osz, 'obstacle_sz', 0.01, 1, valinit=obstacle_size[2])

        ox_slider.on_changed(update_ox_slider)
        oy_slider.on_changed(update_oy_slider)
        oz_slider.on_changed(update_oz_slider)

        osx_slider.on_changed(update_osx_slider)
        osy_slider.on_changed(update_osy_slider)
        osz_slider.on_changed(update_osz_slider)
   

    plt.tight_layout()
    plt.show()
    