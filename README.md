# Object-manipulation-and-task-planning-
This repository is used to make assignments in Object manipulation and task planning.

# lecture 1 Building a robot simulation environment in ros

we have learned how to use URDF and visualize the object interaction with RViz.
Then we toke the omtp_factory.xacro by following the tutorial we end with new file which changes it in to a .urdf file.

To launch our omtp_factory.xacro in gazebo11 and not RViz we had to modify our file visualize_omtp_factory.launch.

which you can see by finding <!-- Include Gazebo launch --> in the file and <!-- Spawn URDF model into Gazebo -->


We have looked in to the OMTP factory world one of the main problem with it was it lagged our PC so we had to remove the walls. We did this by going in too the omtp_factory.xacro and removed  <!-- Fib14 building -->.

This made it easier for us to rebuild the Omtp factory world which we have add a franka panda robot arm and add smart lab modules to visualize the lab and robot arm all one need to do is to open the terminal do is:

1. cd ~/Desktop/Object-manipulation-and-task-planning-/workspace
2. source devel/setup.bash
3. cd ~/Desktop/Object-manipulation-and-task-planning-/workspace/src/omtp_course/omtp_lecture1/launch
4. roslaunch visualize_omtp_factory.launch





# lecture 2 Manipulation with moveit

In this lecture we use Lecture 1 omtp_factory.xacro to steup the moveit by using the following in the terminal:

roslaunch moveit_setup_assistant setup_assistant.launch

A MoveIt Setup Assistant well come up which you can pick your omtp_factory.xacro file to use follow the linke to setup the Moveit. https://ros-planning.github.io/moveit_tutorials/doc/setup_assistant/setup_assistant_tutorial.html


After the setup of the moveit we need to go in to ros_controllers to change the P, d and keep the i the same.

The reason for this it helps to improve the control performance, adapting to phyical characteristics, stability and responsiveness.

To control the Panda_arm we can use the moveit commander or use a pyhon script to control the Panda_arm.


For the pick and place a script as been made with the help of the lectre one of the main thing to note we need hardcode the position of the box since we do not have any sensor on the robot.

to launch the code we need to first start up Rviz and gazebo by using:
roslaunch omtp_factory_moveit_config demo_gazebo.launch

before runing any python code we need to do this
chmod +x /home/wallat/Desktop/workspace/src/omtp_course/omtp_lecture2/scripts/lec2_spawn_box.py

Then we can do:
rosrun omtp_lecture2 lec2_spawn_box.py



# Lecture 3 object detection and grasping

In this lecture we learned about how to use a logical camera which helped to detection in the object we want to fine.

In oure case we want to fine the box that we spowed in the Gazebo11 there is also a banana.

we also learned how to use the tf2 to make a transformtion frames so the robot can fine the box and try to graps it.

In the omtp_lecture3 you well fine the logical_camera the ros_controler in this case where you can change the PID of it also the scripts that use to move the panda arm in Gazebo11.

The script called Tf:object:pose.py in it you well fine all the imports and the transformation with the camera and movit that used to move the panda arm.

If any changes need less say if we need new camera you can just go under urdf and put the camera name and location that been use in Gazebo11 to use.

To run the program two launch file has been made one named omtp_lecture3.launch whuch runs Gazebo11 RVis and makes the world that the robot and everything else that spowned in.

The next launch file makes it easier to run the code so you dont need to naviget all the way to script folder just to do rosrun omtp_lecture3 so instad we made a launch file that called franka_pick_and_place.lanud all it does it runs the script.


Terminal:
roslaunch omtp_lecture3 omtp_lecture3.launch


Then make new Terminal
roslaunch omtp_lecture3 franka_pick_and_place.launch 


If there are any error dont forget to do 
catkin clean
Catkin build

if it does not work then reinstal catkin.







# Lecture 4 Behavior Design with state machines

We used the FlexBE app so we can create machines that perform multiple tasks without the need to hardcode them. To use the FlexBE app, we need to install the following:

1.flexbe_app
2.flexbe_behavior_engine
3.generic_flexbe_state
4.omtp_factory_behaviors
5.panda_v1
6.aau_lab_ros_modules
7.franka_description
8.omtp_support

To ensure that the FlexBE app recognizes the behaviors, we need to do a catkin build. First, we need to run roslaunch omtp_lecture4 omtp_flexbe_demo.launch. Next, we launch the FlexBE app by using roslaunch flexbe_app flexbe_full.launch.

When you launch flexbe_full, the first thing we do is add an overview like the name, package descriptions, etc. Next, we go into the state machine editor and start adding states that our robot needs to perform. Once everything has been added, we do auto-connect. If everything goes well with no errors, we get to see the start-to-finish behavior of the robot.








# Lecture 5 image processing in robotics applications










# Lecture 6 visual recognition for robotics grasping applications















# Lecture 7+8 Guest lecture- robots in contact - from task demonstration to execution in contact

# simple_dmp

This is an (almost) minimal implementation of DMPs in Cartesian space (both position and quaternion) and in Joint space, with an optional (default disabled) roto-dilatation term.


## Dependencies
This project depends on:
* NumPy
* NumPy-Quaternion
* Matplotlib
* Pandas
* dmp
* roboticstoolbox
* spatialmath-python 
* mujoco
* glfw

To install these, run <code>pip3 install --user numpy numpy-quaternion matplotlib pandas</code>.	

## Running
To the run the project, navigate to the root directory and call <code>python3 main.py</code>. This will first train and plot a DMP in joint space and then do the same in Cartesian space for the provided demonstration file, demonstration.csv.


## First setups
• Clone the repository git@gitlab.com:Kramberger/omtp_2024_examples_and_exercises.git.
• In the folder under dmp/dmp_joint.py open the file.
• The file acts as a blueprint in which some of the equation and definitions are missing.

### Exercise 1
This script illustrates the utilization of the JointDMP class from the dmp_joint module for generating and adapting trajectories of robot joints. It imports a trajectory from a CSV file, then employs this trajectory to train a DMP. Subsequently, it simulates the behavior of this DMP and visualizes the outcomes. The script also involves defining the time step dt, computing the total duration tau, and determining the time steps ts.

dt = 1 / 100
tau = len(demo) * dt
ts = np.arange(0, tau, dt)


- Initialize the DMP object with the desired parameters:


dmp_q = dmp_joint.JointDMP(NDOF=7, n_bfs=100, alpha=48, beta=12, cs_alpha=cs_alpha)


- Simulate the DMP and obtain the joint positions, velocities, and accelerations:

- Modify the duration and goal position of the DMP to compare the original and adapted trajectories:



### Exercise 2
This Python script showcases how Dynamic Movement Primitives (DMPs) are applied to control the UR 5 Robot in both joint and Cartesian spaces. It loads a file named "demonstration.csv" that includes data on joint positions, Cartesian coordinates, and orientations. The script processes this data to train and create trajectories using DMPs, and subsequently displays these trajectories on plots for analysis.


### Exercise 3
This script will open a mujoco simulation with a franka robot.The robot joints will be controlled to move like a JTrajectory.csv file using DMP.

    Exercise3_demo_DMP_franka.py 
    

Outputs of the exercises are in the omtp_lecture7/Exercise_Outputs . 


### Exercise 4
to make sure that the DMP_franka.py sends the joint trajectories to gazebo simulation using ROS, we need to make sure that ROS python client libraries have the setups.


1. we need to make sure that our ROS Noetic Python libraries is installed.
2. we need to initialize a ros node.
3. we need to create a publisher.
4. we need to modify the step function so we can calculate new joint states and publish them in to gazebo using ROS massaging system.



### Exercise 5

In here we open the folder named UR5_Example and open the script Grasp_sim.py and we modify the script so it can replace the point to point interpolation function (Jmove and Cmove) with a funtion for linear DMPs and plan the paths between the given points. 





##lecture 9 Deep reinforcement learning for robot control

















##lecture 10 Deep reinforcement learning part 2




