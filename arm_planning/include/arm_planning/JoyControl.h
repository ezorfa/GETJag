/*
 * JoyControl.h
 *
 *  Created on: Jun 12, 2018
 *      Author: Mohammed Afroze
 *
 *  This header file contains the class which commands the arm to move according to
 *  the joystick operation.
 */

#ifndef ARM_PLANNING_JOY_CONTROL_H
#define ARM_PLANNING_JOY_CONTROL_H

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>

#include <arm_planning/Planning.h>
#include <arm_planning/Executor.h>
#include <arm_planning/GoHomeAction.h>
#include <handling_msgs/JoyControlArmWristReset.h>

class JoyControl
{
	public:
		JoyControl (ros::NodeHandle& nh, std::shared_ptr<ArmControlParameterList>& paramList, std::shared_ptr<Executor>& exe, std::shared_ptr<Planning>& plan, std::shared_ptr<JointStateSubscriber>& jointStates);
		~JoyControl () = default;

	private:
		void joySubCB (const sensor_msgs::JoyConstPtr&);
		void setSpeed (const std::string jointName, const double speed);
		void armWristReset (const handling_msgs::JoyControlArmWristResetConstPtr& ptr = nullptr);
		void manualControl ();
		void reset (const handling_msgs::JoyControlArmWristResetConstPtr& ptr = nullptr);

		Executor::Ptr exe;
		Planning::Ptr plan;
		JointStateSubscriber::Ptr jointState;
		ArmControlParameterList::Ptr paramList;
		Parameters staticParams;
		JoystickMappings joyMappings;
		ros::Subscriber joySub;
		std::unique_ptr<actionlib::SimpleActionClient<handling_msgs::GoapMsgAction>> goHomeClient;
		std::map<std::string, ros::ServiceClient> setSpeedService;
		std::map<std::string, double> targetJointAngles;
		std::map<int, ros::Subscriber> jointStateSubs;
		std::map<std::string, std::string> controllerMap;
		std::vector<std::string> jointNames;
		std::map<std::string, double> joyIncrements;
		std::string gripper, speedTopic, gripperController;

		bool resetTarget = false;
		const int ACTIVE_ZONE = 0.9;
		double diffShoulder = 0;
		double diffElbow = 0;
		double diffBase = 0;
		double diffGripper = 0;
		double diffPitch = 0;
		double diffRoll = 0;
		double diffYaw = 0;

};

#endif
