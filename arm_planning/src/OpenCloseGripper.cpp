/*
 * OpenCloseGripperAction.cpp
 *
 *  Created on: Feb 7, 2019
 *      Author: Mohammed Afroze
 */

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <yaml-cpp/yaml.h>

#include <arm_planning/OpenCloseGripper.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>

OpenCloseGripperAction::OpenCloseGripperAction (ros::NodeHandle& nh, std::shared_ptr<ArmControlParameterList>& paramList, std::shared_ptr<Executor>& exe)
		: nh (nh), exe (exe), paramList (paramList)
{
	staticParams = paramList->getStaticParams ();
	server = std::make_unique<GripperServer> (nh, staticParams.gripperActionName, false);
	server->registerGoalCallback (boost::bind (&OpenCloseGripperAction::goalCB, this));
	server->registerPreemptCallback (boost::bind (&OpenCloseGripperAction::preemptCB, this));
	pub = nh.advertise<sensor_msgs::Joy> ("joy", 10, true);
	server->start ();
	ROS_INFO_STREAM("Gripper Action has been started successfully!");
}

OpenCloseGripperAction::~OpenCloseGripperAction ()
{
	server->shutdown ();
}

void OpenCloseGripperAction::goalCB ()
{
	ROS_INFO_STREAM("Command to open/close gripper is received");

	std::string goapMsg = server->acceptNewGoal ()->input;
	paramList->setExecutionMode (ArmControlParameterList::ExecutionMode::Automatic);

	auto currentMode = paramList->getJoystickControlMode ();
	joyMappings = paramList->getJoyMappings ();

	// The following is the joy msg for opening/closing gripper in manual mode.
	sensor_msgs::Joy joyMsg;
	joyMsg.header.stamp.sec = ros::Time::now ().toSec ();
	std::vector<int> btns_open
	{0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0};
	std::vector<int> btns_close
	{0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0};
	std::vector<float> axes
	{0, 0, 0, 0, 0, 0, 0, 0}; // sensor msg ends.

	YAML::Node yaml = YAML::Load (goapMsg);
	std::string command = yaml["gripper"].as<std::string> ();
	if (command == "open")
	{
		if (currentMode == ArmControlParameterList::JoyControlMode::MANUAL)
		{
			paramList->setExecutionMode (ArmControlParameterList::ExecutionMode::Joystick);

			joyMsg.axes = axes;
			joyMsg.buttons = btns_open;

			pub.publish (joyMsg);
		}
		else
		{
			exe->setupGripperControl (ArmControlParameterList::OPEN, staticParams.gripperOpenAngle, this);
			paramList->setGripperMode (ArmControlParameterList::OPEN);
		}
	}
	else
	{
		if (currentMode == ArmControlParameterList::JoyControlMode::MANUAL)
		{
			paramList->setExecutionMode (ArmControlParameterList::ExecutionMode::Joystick);

			joyMsg.axes = axes;
			joyMsg.buttons = btns_close;

			pub.publish (joyMsg);
		}
		else
		{
			exe->setupGripperControl (ArmControlParameterList::CLOSE, 0, this);
			paramList->setGripperMode (ArmControlParameterList::CLOSE);
		}
	}

}
void OpenCloseGripperAction::preemptCB ()
{
	paramList->setExecutionMode (ArmControlParameterList::ExecutionMode::Joystick);
	server->setPreempted ();
}

void OpenCloseGripperAction::setFeedback (double fb)
{

}

void OpenCloseGripperAction::setSucceeded ()
{
	ROS_INFO_STREAM("gripper is opened/closed ");
	setFeedback (100.00);
	handling_msgs::GoapMsgActionResult result;
	result.result.result = "complete";
	server->setSucceeded (result.result, "goal reached");
	paramList->setExecutionMode (ArmControlParameterList::ExecutionMode::Joystick);
}

void OpenCloseGripperAction::setPreempted ()
{
	preemptCB ();
}

bool OpenCloseGripperAction::isActive ()
{
	return server->isActive ();
}
