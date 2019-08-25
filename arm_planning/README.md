# Description
The src folder contains three main nodes : Planning, Executor and JoyControl.

* **Planning** : Uses Moveit! to plan trajectories.

* **Executor** : Executes the generated trajectory from planning node.

* **JoyControl** : This node contains implementation to deal with operator control with joystick.

Also, The following action classes are implemented that uses Planning and Executor nodes to plan a sequence of actions to complete a task:

* **GrabAction** : Executes sequence of actions to grasp an object, when a pose of the object to be grabbed is received.

* **DropAction** : Implements dropping action of the object after an object has been grabbed.

* **InspectAction** : Executes sequence of actions to move the arm to inspect position, when a pose of the object is received.

* **ExtractAction** : Executes sequence of actions to extract, when a pose of the object is received.

* **OpenCloseGripperAction** : Open / Close Gripper functionality is implemented here.

=> This package also contains ArmControlParameterList class that bridges ROS param server and different nodes. It also serves to provide setter/getter methods to set/get dynamic changes in the parameters into/from the yaml file.

# Usage and Commands to startup Arm Motion Planning

The following commands are to be executed in sequence. Make sure that a single command is fully running before executing the next one:

* roslaunch robocup_launch getjag.launch 			 => Start the robot
* roslaunch getjag_moveit_config moveit.launch		 => Launch MoveIt!
* roslaunch arm_planning motion_planning.launch			 => Main Launch File, starts init.cpp node
* roslaunch getjag_moveit_config_new joystick_control.launch     => Joystick Operation
* roslaunch operator_interface test.launch                       => Operator Interface
