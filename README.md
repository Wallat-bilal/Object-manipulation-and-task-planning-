# Object-manipulation-and-task-planning-
This repository is used to make assignments in Object manipulation and task planning.

lecture 1 Building a robot simulation environment in ros

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





lecture 2Manipulation with moveit

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



Lecture 3 object detection and grasping

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







Lecture 4 Behavior Design with state machines

















Lecture 5 image processing in robotics applications










Lecture 6 visual recognition for robotics grasping applications















Lecture 7+8 Guest lecture- robots in contact - from task demonstration to execution in contact










lecture 9 Deep reinforcement learning for robot control

















lecture 10 Deep reinforcement learning part 2




