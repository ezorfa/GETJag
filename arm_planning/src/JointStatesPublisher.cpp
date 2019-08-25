/*
 * JointStatePublisher.cpp
 *
 *  Created on: Feb 9, 2019
 *      Author: mohammed
 */
#include <dynamixel_msgs/JointState.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <arm_planning/ArmControlParameterList.h>

std::map<std::string, double> jointStates;
ros::Publisher pub;
sensor_msgs::JointState jointStateMsg;

void callback (const dynamixel_msgs::JointStateConstPtr& jsm)
{
	for (int i = 0; i < jointStateMsg.name.size (); i++)
	{
		if (jsm->name.compare (jointStateMsg.name[i]) == 0)
		{
			jointStateMsg.position[i] = jsm->current_pos;
		}
	}
	jointStateMsg.header.stamp = ros::Time::now ();
	pub.publish (jointStateMsg);
}

int main (int argc, char **argv)
{
	ros::init (argc, argv, "joint_states_publisher");
	ros::NodeHandle nh;

	//~ ArmControlParameterList paramList(nh);
	//~ std::vector<ros::Subscriber> subs;
	//~ auto joints = paramList.getAllJoints ();
	//~ auto controllers = paramList.getControllers ();

	//~ for (auto joint : joints)
	//~ {
	//~ jointStateMsg.name.push_back (joint);
	//~ subs.push_back(nh.subscribe<dynamixel_msgs::JointState> (controllers[joint] + paramList.getStaticParams().stateTopic, 1, callback));
	//~ }

	jointStateMsg.name.push_back ("GETjag/arm_gripper_joint");
	jointStateMsg.name.push_back ("GETjag/arm_base_joint");
	jointStateMsg.name.push_back ("GETjag/arm_shoulder_joint");
	jointStateMsg.name.push_back ("GETjag/arm_elbow_joint");
	jointStateMsg.name.push_back ("GETjag/arm_wrist_yaw_joint");
	jointStateMsg.name.push_back ("GETjag/arm_wrist_roll_joint");
	jointStateMsg.name.push_back ("GETjag/arm_wrist_pitch_joint");

	jointStateMsg.position.resize (jointStateMsg.name.size ());

	ros::Subscriber sub1 = nh.subscribe<dynamixel_msgs::JointState> ("/GETjag/arm_base_controller/state", 1, callback);
	ros::Subscriber sub2 = nh.subscribe<dynamixel_msgs::JointState> ("/GETjag/arm_shoulder_controller/state", 1, callback);
	ros::Subscriber sub3 = nh.subscribe<dynamixel_msgs::JointState> ("/GETjag/arm_elbow_controller/state", 1, callback);
	ros::Subscriber sub4 = nh.subscribe<dynamixel_msgs::JointState> ("/GETjag/arm_wrist_yaw_controller/state", 1, callback);
	ros::Subscriber sub5 = nh.subscribe<dynamixel_msgs::JointState> ("/GETjag/arm_wrist_roll_controller/state", 1, callback);
	ros::Subscriber sub6 = nh.subscribe<dynamixel_msgs::JointState> ("/GETjag/arm_wrist_pitch_controller/state", 1, callback);
	ros::Subscriber sub7 = nh.subscribe<dynamixel_msgs::JointState> ("/GETjag/arm_gripper_controller/state", 1, callback);

	pub = nh.advertise<sensor_msgs::JointState> ("joint_states", 10);

	ros::spin ();
	return 0;
}
