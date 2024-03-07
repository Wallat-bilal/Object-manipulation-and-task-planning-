# Object-manipulation-and-task-planning-
This repository is used to make assignments in Object manipulation and task planning.

lecture 1
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


