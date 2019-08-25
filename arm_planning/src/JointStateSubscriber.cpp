/*
 * JointStateSubscriber.cpp
 *
 *  Created on: Feb 9, 2019
 *      Author: mohammed
 */

#include<arm_planning/JointStateSubscriber.h>

JointStateSubscriber::JointStateSubscriber (ros::NodeHandle& nh, std::shared_ptr<ArmControlParameterList>& paramList)
{
	jointNames = paramList->getAllJoints ();
	controllerMap = paramList->getControllers ();
	staticParams = paramList->getStaticParams ();

	int i = 0;
	for (auto& joint : jointNames)
	{
		currentJointStateMap[joint] = jointStates[i];
		jointStateSubs.push_back (nh.subscribe<dynamixel_msgs::JointState> (controllerMap[joint] + staticParams.stateTopic, 1, &JointStateSubscriber::jointStateCB, this));
		i++;
	}
}

void JointStateSubscriber::jointStateCB (const dynamixel_msgs::JointStateConstPtr& state)
{
	currentJointStateMap[state->name ].current_pos = state->current_pos;
	currentJointStateMap[state->name ].error = state->error;
	currentJointStateMap[state->name ].load = state->load;

}

JointState JointStateSubscriber::getCurrentState (const std::string joint)
{
	return currentJointStateMap[joint];
}
