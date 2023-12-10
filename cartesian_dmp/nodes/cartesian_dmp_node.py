#!/usr/bin/env python3

import rospy
from cartesian_dmp.cartesian_dmp_2 import CouplingTermObstacleAvoidance3D, CartesianDMP
from cartesian_dmp.srv import DMPRequest, DMPRequestResponse
from geometry_msgs.msg import Point, Pose, Quaternion, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Header
import numpy as np
import pickle

def cartesianDmpRequest(req):
    # This function gets called when a client sends a request to the service
    rospy.loginfo("Cartesian DMP received a request")

    Y = np.array([[pose.position.x, pose.position.y, pose.position.z, 
                   pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w] for pose in req.demo])
    T = np.linspace(0, req.execution_time, len(Y))

    obstacles = np.array([[point.x, point.y, point.z] for point in req.obstacles])
    obstacles = np.mean(obstacles, axis=0) # test to reduce compute time

    with open('input.pkl', 'wb') as file: 
        # A new file will be created 
        pickle.dump([Y,T, obstacles], file) 
        
    dmp = CartesianDMP(
        execution_time=req.execution_time, dt=req.dt,
        n_weights_per_dim=req.n_weights_per_dim, int_dt=0.0001)

    coupling_term = CouplingTermObstacleAvoidance3D(obstacles, gamma=req.gamma, beta=req.beta)

    dmp.imitate(T, Y, allow_final_velocity=True)
    dmp.configure(start_y=Y[0], goal_y=Y[-1])
    print('Exec time: ',str(req.execution_time))
    path = []
    stamped_path = []
    last_p = Y[0,:]
    last_v = np.zeros(6)
    for i in range(int(req.execution_time / req.dt)):
        print(str(i),'/',str(int(req.execution_time / req.dt)))
        p, v = dmp.step(last_p, last_v, coupling_term=coupling_term)
        last_v = v
        last_p = p
        
        path.append(Pose(position=Point(x=p[0],y=p[1],z=p[2]), orientation=Quaternion(x=p[3],y=p[4],z=p[5],w=p[6])))
        stamped_path.append(PoseStamped(pose=Pose(position=Point(x=p[0],y=p[1],z=p[2]))))#, header=Header(frame_id = "base_link")))
        
    print('Computed')
    
    with open('output.pkl', 'wb') as file: 
        # A new file will be created 
        pickle.dump([path, stamped_path], file) 

    imitated_path = Path()
    imitated_path.header.frame_id = "base_link"
    #imitated_path.header.stamp = rospy.Time.now()
    imitated_path.poses = stamped_path
    imitated_path_pub.publish(imitated_path)

    res = DMPRequestResponse()
    res.path=path

    rospy.loginfo("Cartesian DMP sending back the response")
    return res
   

if __name__ == "__main__":
    rospy.init_node('cartesian_dmp')

    service_name = 'get_cartesian_dmp'

    imitated_path_pub = rospy.Publisher("/imitated_path_avoidance", Path, queue_size=1)

    rospy.Service(service_name, DMPRequest, cartesianDmpRequest)
    rospy.loginfo("Custom Service Server is ready to receive requests.")

    rospy.spin()
