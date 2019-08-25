/*
 * ActionWrapper.cpp
 *
 *  Created on: Feb 5, 2019
 *      Author: mohammed
 */

#include <arm_planning/GoHomeAction.h>

GoHomeAction::GoHomeAction (ros::NodeHandle& nh, std::shared_ptr<ArmControlParameterList>& paramList, std::shared_ptr<Executor>& exe, std::shared_ptr<Planning>& plan)
		: nh (nh),exe (exe), plan (plan), paramList (paramList)
{

	staticParams = paramList->getStaticParams ();
	gripperGoalClient = std::make_unique<actionlib::SimpleActionClient<handling_msgs::GoapMsgAction>> (staticParams.gripperActionName, true);
	server = std::make_unique<GoHomeServer> (nh, staticParams.goHomeActionName, false);
	server->registerGoalCallback (boost::bind (&GoHomeAction::goHomeCB, this));
	server->registerPreemptCallback (boost::bind (&GoHomeAction::goHomePreemptCB, this));
	server->start ();
	ROS_INFO_STREAM("GoHomeAction has been started successfully!");
}

GoHomeAction::~GoHomeAction ()
{
	server->shutdown ();
}

void GoHomeAction::goHomeCB ()
{
	exe->setServer (this);
	auto msg = server->acceptNewGoal()->input;

	ROS_INFO_STREAM("Command to go home is received");

	paramList->setExecutionMode (ArmControlParameterList::ExecutionMode::Automatic);
	moveit_msgs::RobotTrajectory traj;
	if (not plan->planToHome (traj))
	{
		ROS_INFO_STREAM("Error in planning to go home");
		return;
	}
	nh.setParam("currentActionStatus", "Going home...");
	exe->executePlan (traj, this);
}

void GoHomeAction::goHomePreemptCB ()
{
	ROS_INFO_STREAM("Command to home has been preempted");
	server->setPreempted ();
	paramList->setExecutionMode (ArmControlParameterList::ExecutionMode::Joystick);

}

void GoHomeAction::setFeedback (double fb)
{
	feedback.feedback.progress = fb;
	server->publishFeedback (feedback.feedback);
}

void GoHomeAction::setSucceeded ()
{
	nh.setParam("currentActionStatus", "Arm is at home position !!!");
	setFeedback(100);

	ROS_INFO_STREAM("Command to go home has been succeeded. Arm is at Home Position.");
	handling_msgs::GoapMsgActionResult result;
	result.result.result = "completed";
	server->setSucceeded (result.result, "goal reached!");
	paramList->setExecutionMode (ArmControlParameterList::ExecutionMode::Joystick);
}

void GoHomeAction::setPreempted ()
{
	nh.setParam("currentActionStatus", "Going home, preempted !");
	setFeedback(-1);
	goHomePreemptCB ();
}

bool GoHomeAction::isActive ()
{
	return server->isActive ();
}
