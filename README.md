# Final Master's Thesis: Implementation and evaluation of an obstacle avoidance system in a collaborative robot

This work extends the work from [Roy Eriksen](https://github.com/eriksenroy/roy_dmp.git) by adding realtime workspace feedback for collision check and end-effector obstacle avoidance capabilities.

The code uses Dynamic Movement Primitives (DMPs) and Artificial Potential Fields (APFs) to generate an obstacle-avoiding trajectory while following a demo trajectory.

The code is created to work with the UR CB-series robots family and an Intel Realsense D345 depth camera. 

It has been tested with a real UR3 robot.

## Content
* [Setup](#setup)
* [Execution](#execution)
* [Work status](#work-status)
* [Report](#report)


## Setup
### Workspace setup
To create an execution environment:

1. Create a workspace (if needed):

    `mkdir -p <WORKSPACE_NAME>`

    `cd <WORKSPACE_NAME>`

2. Clone the repository:

    `git clone --recursive https://github.com/Izan00/ur_obstacle_avoidance.git src/`

3. Install the needed dependencies:

    `chmod +x src/run.sh`
  
    `./src/run.sh`

4. Compile and source the workspace

    `catkin_make`

    `source devel/setup.bash`

### Robot setup
Generate the robot calibration file with the following: 

  `roslaunch ur_calibration calibration_correction.launch robot_ip:=<IP> target_filename:="${HOME}/ur_ws/my_robot_calibration.yaml"`

  See: [ur_calibration](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_calibration/README.md) for more details.

Install the URCap driver in the robot using the pendant:

* URCap file can be found [here](https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCap/releases)

  Note: Tested the [externalcontrol-1.0.5.urcap](https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCap/releases/download/v1.0.5/externalcontrol-1.0.5.urcap)

* And the installation guide [here](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/install_urcap_cb3.md)

  Note: Set port to 50002 and the IP from the HOST PC
  
  Note: The program file needs to be saved and running for the robot to work

## Execution
To run the application, follow the steps:

1. Start ROS core:
   
    `roscore`

2. Launch the GUI for DMP:

    `python3 src/roy_dmp/script/gui_dmp.py`

    * Set the robot IP or enable the simulation checkbox.
    * Click the **Connect to Robot** button. Once connected, the button text will turn green, and the **Start DMP application** button will activate.

3. Begin the DMP application:

    Click the **Start DMP application** button.

4. Execution with obstacle avoidance:
    * Navigate to the execute window in the GUI.
    * Select the desired demo DMP, start posse, and target pose. Note: The start pose must be the same as the robot's current pose.
    * Activate the realtime obstacle avoidance checkbox.
    * Click the **Generate plan** button.
    * Run the URCap program with the robot pendant.
    * Start the application by clicking the **Execute plan** button.


A detailed explanation of the DMP application execution can be found in the following [report](https://upcommons.upc.edu/bitstream/handle/2117/340118/tfm-roy-eriksen.pdf?sequence=1).



## Work status
- [x] Reproduce Roy's work
  - [x] Simulations
  - [x] Real-life
  
- [x] Add Intel Realsense D345 depth camera
  - [x] Implementations 
    - [x] Simulator 
    - [x] Real-life test
  - [x] Location
    - [x] Fixed external
    - [x] Tool
  
- [x] Feedback mechanism
  - [x] Object detection 
    - [x] PointCloud
      - [x] DBSCAN Clustering (removed)
      - [x] Euclidean Clustering
    - [ ] Color camera reprojection
      - [ ] Traditional CV methods
      - [ ] NN methods
    - [ ] Color camera + Pointcloud matching  
      - [ ] Traditional CV methods
      - [ ] NN methods
  - [x] Object virtualization
    - [ ] Bounding box
    - [ ] Image
    - [x] Mesh
  
- [x] Collision check
  - [x] URFD (Moveit)
  - [x] Planing scene
  - [x] Path intersection  
  - [x] Collsion distance
   
- [x] Obstacle avoidance stages
  - [x] Collision handling
    - [x] Object movement check (stop at any movement)
    - [x] Realtime planing scene collision (ignore movement that does not interfere)
  - [x] Safe stop
  - [x] Path recomputation
    - [x] DMP + APF
    - [ ] DMP + APF + RL
  - [x] Realtime response
  - [x] Implementations 
    - [x] Simulator 
    - [x] Real-life test

- [ ] Optimizations
  - [x] Reduce point-cloud density
  - [x] Filter ground/workspace outside 
  - [ ] Limit realtime collision check horizon 
  - [ ] Integrate the Avoidance parameters in the GUI

## Report
The [thesis](TFM__Implementation_and_evaluation_of_an_obstacle_avoidance_system_in_a_collaborative_robot_compressed.pdf) of this work can be found in the repository root folder.
