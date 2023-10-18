# TFM
## Implementation and evaluation of an obstacle avoidance system in a collaborative robot

### Introduction


### Setup
#### Workspace setup
Create a workspace (if needed):

  `mkdir -p <WORKSPACE_NAME>`

  `cd <WORKSPACE_NAME>`

Clone the repository:

  `git clone --recursive https://github.com/Izan00/ur_obstacle_avoidance.git src/`

Install the needed dependencies:

  `chmod +x src/run.sh`
  
  `./src/run.sh`

Compile and soruce the workspace

  `catkin_make`

  `source devel/setup.bash`

#### Robot setup
Cenerate the robot calibration file with: 

  `roslaunch ur_calibration calibration_correction.launch robot_ip:=<IP> target_filename:="${HOME}/ur_ws/my_robot_calibration.yaml"`

  See: `https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_calibration/README.md`

Install the URCap dirver in the robot using the pendant:

* URCap file can be found [here](https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCap/releases)

  Note: Tested the [externalcontrol-1.0.5.urcap](https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCap/releases/download/v1.0.5/externalcontrol-1.0.5.urcap)

* And the installation guide [here](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/install_urcap_cb3.md)

  Note: Set port to 50002 and the IP from the HOST PC
  
  Note: Program file needs to be saved a running for the robot to work

### Execution


### Work status
- [x] Reproduce Roy work
  - [x] Simulations
  - [x] Real-life
- [x] Add Intel Realsense 345i camera
  - [x] Implementations 
    - [x] Simulator 
    - [x] Real-life test
  - [x] Location
    - [x] Fixed external
    - [x] Tool
- [ ] Obstacle tracking
  - [x] Obstacle definition
  - [ ] Tracking methods 
    - [x] PointCloud
      - [ ] DBSCAN Clustering / Segmentation + Matching
      - [x] DBSCAN Clustering / Segmentation + Mesh creation
    - [ ] Color camera reprojection
      - [ ] Traditional CV methods
      - [ ] NN methods
    - [ ] Color camera + Pointcloud matching  
      - [ ] Traditional CV methods
      - [ ] NN methods
- [ ] Collision check
  - [x] URFD (Moveit)
  - [x] Planing scene
  - [ ] Path intersection  


- [ ] Obstacle avoidance stages
  - [ ] Collsion 
    - [x] Object movement check (stop at any movement)
      or 
    - [ ] Real-time planing scene collision (ignore movemetn that not interferes)
  - [x] Safe stop
  - [ ] Path recomputation
  - [ ] Real-time implementation

- [ ] Optimizations
  - [x] Reduce pointcloud density
  - [x] Filter ground/workare outside 
  - [ ] Limit realtime collision check horizon 

Links to Study:

- 3D bounding box projection
	* https://github.com/IntelligentRoboticsLabs/gb_visual_detection_3d

- DPMs w/obstacle avoidance
	* https://github.com/mathiasesn/obstacle_avoidance_with_dmps

- RL for obstacle avoidance

	* https://www.mdpi.com/2076-3417/12/13/6629
	* https://towardsdatascience.com/training-of-robotic-manipulators-on-the-obstacle-avoidance-task-through-reinforcement-learning-ea2a3404883f
	* https://arxiv.org/pdf/2301.05980.pdf

