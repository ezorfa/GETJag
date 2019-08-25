/*
 * JointStateSubscriber.h
 *
 *  Created on: Feb 9, 2019
 *      Author: mohammed
 *
 *  This class subscribes to the joint states and stores in the form of the map. Users can use getCurrentState(joint) function
 *  to query the current state of the specific joint, its load and error.
 */

#ifndef ROS_PG_WS_2018_19_ARM_PLANNING_INCLUDE_ARM_PLANNING_JOINTSTATESUBSCRIBER_H_
#define ROS_PG_WS_2018_19_ARM_PLANNING_INCLUDE_ARM_PLANNING_JOINTSTATESUBSCRIBER_H_

#include <dynamixel_msgs/JointState.h>
#include <ros/ros.h>

#include <arm_planning/ArmControlParameterList.h>

struct JointState
{
		std::string name;
		double current_pos;
		double error;
		double load;
};

class JointStateSubscriber
{
	public:
		JointStateSubscriber (ros::NodeHandle& nh, std::shared_ptr<ArmControlParameterList>& paramList);
		~JointStateSubscriber () = default;
		JointState getCurrentState (const std::string joint);
		using Ptr = std::shared_ptr<JointStateSubscriber>;

	private:
		void jointStateCB (const dynamixel_msgs::JointStateConstPtr& state);
		JointState baseJointState, shoulderJointState, elbowJointState, wristPitchJointState, wristYawJointState, wristRollJointState, gripperJointState;
		Parameters staticParams;
		std::shared_ptr<ArmControlParameterList> paramList;
		std::map<std::string, std::string> controllerMap;
		std::vector<std::string> jointNames, controllers;
		std::vector<ros::Subscriber> jointStateSubs;
		std::map<std::string, JointState> currentJointStateMap;
		std::string robotNamespace, gripperController;
		std::map<std::string, std::string> jointControllers;
		std::vector<JointState> jointStates =
		{baseJointState, elbowJointState, shoulderJointState, wristPitchJointState, wristRollJointState, wristYawJointState, gripperJointState};

};

#endif /* ROS_PG_WS_2018_19_ARM_PLANNING_INCLUDE_ARM_PLANNING_JOINTSTATESUBSCRIBER_H_ */
