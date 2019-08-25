/*
 * DropObjectAction.cpp
 *
 *  Created on: Feb 7, 2019
 *      Author: mohammed
 */

#include <vector>
#include <yaml-cpp/yaml.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <arm_planning/DropAction.h>

DropObjectAction::DropObjectAction (ros::NodeHandle& nh, std::shared_ptr<ArmControlParameterList>& paramList, Planning::Ptr& plan, std::shared_ptr<Executor>& exe)
		: nh (nh), exe (exe), paramList (paramList), plan (plan)
{
	staticParams = paramList->getStaticParams ();
	server = std::make_unique<DropServer> (nh, staticParams.dropActionName, false);
	gripperGoalClient = std::make_unique<actionlib::SimpleActionClient<handling_msgs::GoapMsgAction>> (staticParams.gripperActionName, true);
	client = nh.serviceClient<handling_msgs::ToggleGUIExecuteCancelBtn>(staticParams.GUIToggleExeCancel);
	server->registerGoalCallback (boost::bind (&DropObjectAction::goalCB, this));
	server->registerPreemptCallback (boost::bind (&DropObjectAction::preemptCB, this));
	server->start ();

	ROS_INFO_STREAM("Drop Action has been started successfully!");
}

DropObjectAction::~DropObjectAction ()
{
	server->shutdown ();
}

void DropObjectAction::goalCB ()
{
	ROS_INFO_STREAM("Command to Drop object is received");

	std::string goapMsg = server->acceptNewGoal ()->input;

	paramList->setExecutionMode (ArmControlParameterList::ExecutionMode::Automatic);

	// get the object which was set by Grab action server.
	std::vector<std::string> objectID;
	objectID.push_back (plan->getGrabbedObject ());

	nh.setParam("currentActionStatus", "Preparing to drop object...");
	setFeedback (15);

	handling_msgs::GoapMsgAction goal;
	std::string goalString = "{ gripper : open}";
	goal.action_goal.goal.input = goalString;
	gripperGoalClient->sendGoal (goal.action_goal.goal);

	// detach the object first from the tool tip and then remove it from the planning scene.
	if (not objectID.empty ())
	{
		plan->dettachObjectFromTool (objectID.front ());
		ros::Duration (2).sleep ();
		nh.setParam("currentActionStatus", "Object is dropped, Action completed !!!");
		setFeedback (100.00);
		plan->removeCollisionObjects (objectID);
		plan->removeCollisionObjects (objectID);
	}

	handling_msgs::ToggleGUIExecuteCancelBtn toggleReq;
	toggleReq.request.enableExecuteBtn = true;
	client.call(toggleReq);

	// delay is necessary to completely remove the object from the planning scene.
	ros::Duration (3).sleep ();

	setSucceeded ();

}
void DropObjectAction::preemptCB ()
{
	nh.setParam("currentActionStatus", "Preempted...Goal unsuccessful!");
	setFeedback (-1);

	handling_msgs::ToggleGUIExecuteCancelBtn toggleReq;
	toggleReq.request.enableExecuteBtn = true;
	client.call(toggleReq);

	paramList->setExecutionMode (ArmControlParameterList::ExecutionMode::Joystick);
	server->setPreempted ();
}

void DropObjectAction::setFeedback (double fb)
{
	handling_msgs::GoapMsgActionFeedback feedback;
	feedback.feedback.progress = fb;
	server->publishFeedback (feedback.feedback);
}

void DropObjectAction::setSucceeded ()
{
	handling_msgs::GoapMsgActionResult result;
	result.result.result = "complete";
	server->setSucceeded (result.result, "object dropped");
	paramList->setExecutionMode (ArmControlParameterList::ExecutionMode::Joystick);
}

void DropObjectAction::setPreempted ()
{
	preemptCB ();
}

bool DropObjectAction::isActive ()
{
	return server->isActive ();
}
