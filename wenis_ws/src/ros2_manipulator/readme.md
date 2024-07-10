# Test Manipulator

This repository includes the simulations made
for the SVTRP CTEx Fase 1 for testing purposes.
The main goal is starting an environment for testing
purposes in wich the main goal is using this arm for
explosives disarming operations and chemical,
biological, radiological and nuclear verification. 

Unfortunately, the gripper is not working correctly but as soon I get some
time I'll fix it.

---
### System Especifications
- Ubuntu 22.04
- Python 3.10
- ROS Humble
- Gazebo Classic 11

## Usage

### Launch the simulation

To launch the simulation run:


     ros2 launch arm_pkg main_arm.launch.py

     ros2 launch arm_pkg start_moveit.launch.py


These commands will: 
- start gazebo classic;
- run the urdf package and the controllers;
- spawn the robot in gazebo;
- launch the customized move_group; and
- launch the custom RVIz.

### Moving the arm with scripts

In order to use a c++ file as a client to move the 
manipulator you need to run the scripts in the 
moveit2_scripts package. 

### Cartesian Movement

In order to move the robot, using the c++ scripts but
in a Cartesian trajectory you will need to run
the file in the moveit2_cartesian package.

### Perception
We can also make a simple perception test, using
the simple_grasping_ros2 node. In order to do that 
you need to start the basic perception nodel with:

    ros2 run simple_grasping basic_grasping_perception_node --ros-args -p debug_topics:=true

we need to add something for the algorithm to perceive
and for that will add two boxes:

    ros2 run gazebo_ros spawn_entity.py -file PATH_TO_WORKSPACE/src/grasp_box2.urdf -x 0.5 -y 0.26 -z 0.1 -entity grasp_box2
    ros2 run gazebo_ros spawn_entity.py -file PATH_TO_WORKSPACE/src/grasp_box.urdf -x 0.5 -y 0.13 -z 0.1 -entity grasp_box

Now you can run the client node and get the objects position:

    ros2 run get_pose_client get_pose_client

### To Do

[ ] Fix the gripper

[ ] Understand why sometimes the planning doesn't work

[ ] Add OpenCV to pick and place