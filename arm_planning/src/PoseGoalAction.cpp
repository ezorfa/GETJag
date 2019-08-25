/*
 * PoseGoalAction.cpp
 *
 *  Created on: Feb 7, 2019
 *      Author: Mohammed Afroze
 */

#include <arm_planning/PoseGoalAction.h>

PoseGoalAction::PoseGoalAction (ros::NodeHandle& nh, std::shared_ptr<ArmControlParameterList>& paramList, std::shared_ptr<Executor>& exe, std::shared_ptr<Planning>& plan)
		: nh(nh),exe (exe), plan (plan), spinner (1, &callbackQueue), paramList (paramList)
{
	using moveit = moveit::planning_interface::MoveGroupInterface;
	staticParams = paramList->getStaticParams ();
	client = nh.serviceClient<handling_msgs::ToggleGUIExecuteCancelBtn>("/dexterity/GUI/toggle_exe_cancel");
	server = std::make_unique<PoseGoalServer> (nh, staticParams.goalActionName, false);
	server->registerGoalCallback (boost::bind (&PoseGoalAction::poseGoalCB, this));
	server->registerPreemptCallback (boost::bind (&PoseGoalAction::poseGoalPreemptCB, this));
	server->start ();

	ROS_INFO_STREAM("GoalPose Action has been started successfully!");
}

PoseGoalAction::~PoseGoalAction ()
{
	server->shutdown ();
}
void PoseGoalAction::poseGoalCB ()
{
	ROS_INFO_STREAM("Command to pose goal is received");

	if (exe->isBusy ())
	{
		exe->abortExecution ();
	}

	geometry_msgs::Pose pose;
	std::string goapMsg = server->acceptNewGoal ()->input;

	// Conversion of goap messgase to geometric::Pose msg.
	YAML::Node yaml = YAML::Load (goapMsg);
	pose.position.x = yaml["$activeObject.atPos"][0].as<double> ();
	pose.position.y = yaml["$activeObject.atPos"][1].as<double> ();
	pose.position.z = yaml["$activeObject.atPos"][2].as<double> ();

	pose.orientation.x = yaml["$activeObject.atPos"][3].as<double> ();
	pose.orientation.y = yaml["$activeObject.atPos"][4].as<double> ();
	pose.orientation.z = yaml["$activeObject.atPos"][5].as<double> ();
	pose.orientation.w = yaml["$activeObject.atPos"][6].as<double> ();

	paramList->setExecutionMode (ArmControlParameterList::ExecutionMode::Automatic);

	moveit_msgs::RobotTrajectory traj;

	// Send text feedback on the ROS param currentActionStatus for GUI to print the message.
	nh.setParam("currentActionStatus", "starting planning for the action...");
	setFeedback (5);

	if (not plan->plan (pose, traj, true))
	{
		ROS_INFO_STREAM("Error in Planning");
		poseGoalPreemptCB ();
		return;
	}

	nh.setParam("currentActionStatus", "Going to a goal pose...");
	setFeedback (25);

	exe->executePlan (traj, this);

}

void PoseGoalAction::poseGoalPreemptCB ()
{
	nh.setParam("currentActionStatus", "Preempted...Goal unsuccessful!");
	setFeedback (-1);

	handling_msgs::ToggleGUIExecuteCancelBtn toggleReq;
	toggleReq.request.enableExecuteBtn = true;
	client.call(toggleReq);

	ROS_INFO_STREAM("Command to pose goal has been preempted");

	handling_msgs::GoapMsgActionResult result;
	result.result.result = "false";
	server->setPreempted (result.result, "planning failed!");
	paramList->setExecutionMode (ArmControlParameterList::ExecutionMode::Joystick);
}

void PoseGoalAction::setFeedback (double fb)
{
	feedback.feedback.progress = fb;
	server->publishFeedback (feedback.feedback);
}

void PoseGoalAction::setSucceeded ()
{
	nh.setParam("currentActionStatus", "Succeeded...Arm is at goal pose !!!");
	setFeedback (100.00);

	handling_msgs::ToggleGUIExecuteCancelBtn toggleReq;
	toggleReq.request.enableExecuteBtn = true;
	client.call(toggleReq);

	ROS_INFO_STREAM("Command to goal pose has been succeeded.");

	handling_msgs::GoapMsgActionResult result;
	result.result.status = "succeeded";
	server->setSucceeded (result.result, "goal reached");
	paramList->setExecutionMode (ArmControlParameterList::ExecutionMode::Joystick);

}
void PoseGoalAction::setPreempted ()
{
	poseGoalPreemptCB ();
}

bool PoseGoalAction::isActive ()
{
	return server->isActive ();
}

