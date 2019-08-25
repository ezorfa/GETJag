/*
 * GrabAction.cpp
 *
 *  Created on: Feb 7, 2019
 *      Author: Mohammed Afroze
 */

#include <yaml-cpp/yaml.h>
#include <std_msgs/Bool.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <arm_planning/GrabAction.h>
#include <handling_msgs/ObjectGrabbed.h>
#include <handling_msgs/ToggleGUIExecuteCancelBtn.h>

GrabAction::GrabAction (ros::NodeHandle& nh, std::shared_ptr<ArmControlParameterList>& paramList, std::shared_ptr<Executor>& exe, std::shared_ptr<Planning>& plan)
		: nh (nh), exe (exe), plan (plan), paramList (paramList), spinner (1, &callbackQueue)
{
	nhPrivate.setCallbackQueue (&callbackQueue);
	spinner.start ();

	staticParams = paramList->getStaticParams ();
	objectGrabbedPub = nhPrivate.advertise<handling_msgs::ObjectGrabbed> (staticParams.objectGrabbedGrasp, 10, true);
	poseGoalClient = std::make_unique<actionlib::SimpleActionClient<handling_msgs::GoapMsgAction>> (staticParams.goalActionName, true);
	gripperGoalClient = std::make_unique<actionlib::SimpleActionClient<handling_msgs::GoapMsgAction>> (staticParams.gripperActionName, true);
	client = nhPrivate.serviceClient<handling_msgs::ToggleGUIExecuteCancelBtn> (staticParams.GUIToggleExeCancel);
	server = std::make_unique<GrabServer> (nhPrivate, staticParams.grabActionName, false);
	server->registerGoalCallback (boost::bind (&GrabAction::goalCB, this));
	server->registerPreemptCallback (boost::bind (&GrabAction::goalPreemptCB, this));
	server->start ();
	listener = std::make_shared<tf::TransformListener> ();

	ROS_INFO_STREAM("Grab Action Pipeline has been started successfully!");
}

GrabAction::~GrabAction ()
{
	server->shutdown ();
}

void GrabAction::doTransform (const geometry_msgs::Pose in, geometry_msgs::Pose &out, const std::string target, const std::string base)
{
	tf::Transform tfPose = tf::Transform (tf::Quaternion (in.orientation.x, in.orientation.y, in.orientation.z, in.orientation.w), tf::Vector3 (in.position.x, in.position.y, in.position.z));

	if ((in.orientation.x == 0.0) && (in.orientation.y == 0.0) && (in.orientation.z == 0.0) && (in.orientation.w == 0.0))
	{
		ROS_ERROR("Quaternion [0,0,0,0] does not exist! The unit quaternion is [0,0,0,1]!");
		ros::Duration (2.0).sleep ();
		return;
	}

	tf::StampedTransform transform;
	try
	{
		ros::Time time = ros::Time::now ();
		listener->waitForTransform (base, target, time, ros::Duration (4.0));
		listener->lookupTransform (base, target, time, transform);
		std::cout << transform.getOrigin ().x () << ", " << transform.getOrigin ().y () << ", " << transform.getOrigin ().z () << " | " << transform.getRotation ().x () << ", " << transform.getRotation ().y () << ", " << transform.getRotation ().z () << ", " << transform.getRotation ().w () << std::endl;
	}
	catch (tf::TransformException ex)
	{
		ROS_ERROR("%s", ex.what ());
		ros::Duration (1.0).sleep ();
		return;
	}

	tfPose = transform.inverse () * tfPose;

	out.position.x = tfPose.getOrigin ().x ();
	out.position.y = tfPose.getOrigin ().y ();
	out.position.z = tfPose.getOrigin ().z ();

	out.orientation.x = tfPose.getRotation ().x ();
	out.orientation.y = tfPose.getRotation ().y ();
	out.orientation.z = tfPose.getRotation ().z ();
	out.orientation.w = tfPose.getRotation ().w ();
}

void GrabAction::goalCB ()
{
	ROS_INFO_STREAM("Command to grab pipeline is received");

	if (exe->isBusy ())
	{
		exe->abortExecution ();
	}

	geometry_msgs::Pose pose;

	std::string goapMsg = server->acceptNewGoal ()->input;

	YAML::Node yaml = YAML::Load (goapMsg);
	pose.position.x = yaml["$activeObject.atPos"][0].as<double> ();
	pose.position.y = yaml["$activeObject.atPos"][1].as<double> ();
	pose.position.z = yaml["$activeObject.atPos"][2].as<double> ();

	pose.orientation.x = yaml["$activeObject.atPos"][3].as<double> ();
	pose.orientation.y = yaml["$activeObject.atPos"][4].as<double> ();
	pose.orientation.z = yaml["$activeObject.atPos"][5].as<double> ();
	pose.orientation.w = yaml["$activeObject.atPos"][6].as<double> ();

	ROS_INFO_STREAM("Original Pose: ");
	ROS_INFO_STREAM(pose);
	
	
	// ******[[ !!! Below lines need to be uncommented if doing integrated tests with all other groups ]]*******
	//Transforming from odom frame into base_link frame  
	//geometry_msgs::Pose before = pose;
	//doTransform (before, pose, "/GETjag/base_link", "/GETjag/odom");
	//ROS_INFO_STREAM("Transformed Pose: ");
	//ROS_INFO_STREAM(pose);

	std::string object = yaml["$activeObject.id"].as<std::string> ();
	objectID.clear ();
	objectID.push_back (object);
	grasps = paramList->getAllGrasps (objectID.front ());

	if (not (yaml["$activeObject.which"].as<std::string> () == "None"))
	{
		ROS_INFO_STREAM("Planning for only selected grasps.");
		std::string which = yaml["$activeObject.which"].as<std::string> ();
		std::map<std::string, std::vector<double>> temp;
		temp[which] = grasps[which];
		grasps.clear ();
		grasps = temp;
	}
	else
	{
		ROS_INFO_STREAM("Planning for all grasps.");
	}

	paramList->setExecutionMode (ArmControlParameterList::ExecutionMode::Automatic);

	std::string meshPath = paramList->getMeshPath (objectID.front ());

	/***************************** Add the object as a collision object ****************************************/

	plan->addMeshCollisionObjects (objectID, meshPath, pose);
	ros::Duration (2).sleep ();

	/***************************** Selection of best feasable grasps *******************************************/

	tf2::Quaternion orientation;

	std::string whichGrasp;
	bool isPlanningSuccessful = false;

	nhPrivate.setParam ("currentActionStatus", "starting planning for the action...");
	setFeedback (5);

	// Test multiple grasps points and attempt the one which can be planned successfully by moveit till approaching the object.
	for (auto const& element : grasps)
	{
		geometry_msgs::Pose tempPose = pose;
		moveit_msgs::RobotTrajectory traj;
		// Adjustments to position the arm properly at the grasping pose.
		tempPose.position.x = (double)tempPose.position.x + (double)element.second[3] + (double)element.second[6] + (double)element.second[9];
		tempPose.position.y = (double)tempPose.position.y + (double)element.second[4] + (double)element.second[7] + (double)element.second[10];
		tempPose.position.z = (double)tempPose.position.z + (double)element.second[5] + (double)element.second[8] + (double)element.second[11];
		orientation.setRPY (-element.second[0], element.second[1], element.second[2]);

		tempPose.orientation = tf2::toMsg (orientation);
		tempPose.orientation.x = -tempPose.orientation.x;

		ROS_INFO_STREAM("which Grasp?");
		ROS_INFO_STREAM(element.first);

		auto tempPose1 = tempPose;
		// This flag is used as an indicator whether to stop the current planning/ execution.
		bool continuePlanning = false;
		// check if the current action has been cancelled or preempted by a different goal.
		if (isCancelRequested ())
			return;

		if (plan->plan (tempPose, traj, true))
		{
			continuePlanning = true;
		}
		else
			continue;

		if (continuePlanning)
		{
			// Just remove collision object temporarily to facilitate plan faster while planning to approach object.
			//plan->removeCollisionObjects (objectID);
			// sleep is required to properly complete removing.
			//ros::Duration (2).sleep ();

			//tempPose1.position.x = (double)tempPose1.position.x - (double)element.second[3] - (double)element.second[12];
			//tempPose1.position.y = (double)tempPose1.position.y - (double)element.second[4] - (double)element.second[13];
			//tempPose1.position.z = (double)tempPose1.position.z - (double)element.second[5] - (double)element.second[14];

			// plan to approach object.
			//if (plan->plan (tempPose1, traj, true))
			//{
				whichGrasp = element.first;
				isPlanningSuccessful = true;
				//plan->addMeshCollisionObjects (objectID, meshPath, pose);
				pose = tempPose;
				ros::Duration (2).sleep ();
				break;
			//}
		}
	}

	handling_msgs::GoapMsgAction goal;

	if (not isPlanningSuccessful)
	{
		handling_msgs::GoapMsgActionResult result;
		result.result.status = "Succeeded";
		server->setSucceeded (result.result, "$activeObject.isCarried:False;$robot.hasObj:None");
//		goalPreemptCB ();
		return;
	}
	else // START PIPELINE EXECUTION !
	{
		nhPrivate.setParam ("currentActionStatus", "Planning completed...");
		setFeedback (20);
		std::string goalString = convertGoalToGoapMsg (pose);
		goal.action_goal.goal.input = goalString;

		/************************************* Move near object *********************************************/

		poseGoalClient->sendGoal (goal.action_goal.goal);
		bool result = poseGoalClient->waitForResult ();
		bool continuePipeline = false;

		if (result)
		{
			if (poseGoalClient->getState ().state_ == actionlib::SimpleClientGoalState::SUCCEEDED)
			{
				nhPrivate.setParam ("currentActionStatus", "Arm is near object...");
				setFeedback (50);
				continuePipeline = true;
			}
		}
		if (isCancelRequested ())
			return;

		/*************************************** Open Gripper **********************************************/

		if (continuePipeline)
		{
			continuePipeline = false;
			handling_msgs::GoapMsgAction goal;
			goal.action_goal.goal.input = "{gripper : open}";
			gripperGoalClient->sendGoal (goal.action_goal.goal);
			bool result = gripperGoalClient->waitForResult ();
			if (result)
			{
				if (gripperGoalClient->getState ().state_ == actionlib::SimpleClientGoalState::SUCCEEDED)
				{
					nhPrivate.setParam ("currentActionStatus", "Gripper is opened...Approaching object...");
					setFeedback (60);
					continuePipeline = true;
				}
			}
		}
		if (isCancelRequested ())
			return;

		/************************************* Approaching Object ********************************************/

		if (continuePipeline)
		{
			continuePipeline = false;
			pose.position.x = pose.position.x - grasps[whichGrasp][3] - grasps[whichGrasp][12];
			pose.position.y = pose.position.y - grasps[whichGrasp][4] - grasps[whichGrasp][13];
			pose.position.z = pose.position.z - grasps[whichGrasp][5] - grasps[whichGrasp][14];

			goal.action_goal.goal.input = convertGoalToGoapMsg (pose);
			poseGoalClient->sendGoal (goal.action_goal.goal);
			bool result = poseGoalClient->waitForResult ();
			if (result)
			{
				if (poseGoalClient->getState ().state_ == actionlib::SimpleClientGoalState::SUCCEEDED)
				{
					continuePipeline = true;
				}
			}
		}
		if (isCancelRequested ())
			return;

		/*************************************** Close  Gripper **********************************************/

		if (continuePipeline)
		{
			continuePipeline = false;
			handling_msgs::GoapMsgAction goal;
			goal.action_goal.goal.input = "{gripper : close}";
			gripperGoalClient->sendGoal (goal.action_goal.goal);
			bool result = gripperGoalClient->waitForResult ();
			if (result)
			{
				if (gripperGoalClient->getState ().state_ == actionlib::SimpleClientGoalState::SUCCEEDED)
				{
					plan->attachObjectToTool (objectID.front ());
					isObjectAttacedToTool = true;
					nhPrivate.setParam ("currentActionStatus", "Object is grabbed, Action completed !!!");
					setFeedback (100);
					setSucceeded ();
				}
			}
		}
		else
		{
			if (isObjectAttacedToTool)
				plan->dettachObjectFromTool (objectID.front ());

			handling_msgs::GoapMsgActionResult result;
			result.result.status = "Succeeded";
			server->setSucceeded (result.result, "$activeObject.isCarried:False;$robot.hasObj:None");
//			goalPreemptCB ();
			return;
		}
	}
}

bool GrabAction::isCancelRequested ()
{
	callbackQueue.callAvailable ();
	if (server->isPreemptRequested ())
	{
		if (exe->isBusy ())
		{
			exe->abortExecution ();
		}
		cleanUp ();
		return true;
	}
	return false;
}

std::string GrabAction::convertGoalToGoapMsg (const geometry_msgs::Pose& pose) const
{
	std::string goapMsg = "{$activeObject.atPos : [" + std::to_string (pose.position.x) + "," + std::to_string (pose.position.y) + "," + std::to_string (pose.position.z) + "," + std::to_string (pose.orientation.x) + "," + std::to_string (pose.orientation.y) + "," + std::to_string (pose.orientation.z) + "," + std::to_string (pose.orientation.w) + "]}";
	std::cout << goapMsg << std::endl;
	return goapMsg;
}

void GrabAction::goalPreemptCB ()
{
//	exe->abortExecution();
	nhPrivate.setParam ("currentActionStatus", "Preempted...Goal unsuccessful!");
	setFeedback (-1);
	handling_msgs::ToggleGUIExecuteCancelBtn toggleReq;
	toggleReq.request.enableExecuteBtn = true;
	client.call (toggleReq);
	handling_msgs::GoapMsgActionResult result;
	result.result.status = "Aborted";
	cleanUp ();
	server->setPreempted (result.result, "planning failed!");
}

void GrabAction::cleanUp ()
{
	ROS_INFO_STREAM("Grab pipeline is not successful :( ");
	if (isObjectAttacedToTool)
		plan->dettachObjectFromTool (objectID.front ());
	if (not objectID.empty ())
	{
		plan->removeCollisionObjects (objectID);
		ros::Duration (2).sleep ();
	}
	paramList->setExecutionMode (ArmControlParameterList::ExecutionMode::Joystick);
}

void GrabAction::setFeedback (double fb)
{
	handling_msgs::GoapMsgActionFeedback feedback;
	feedback.feedback.progress = fb;
	server->publishFeedback (feedback.feedback);
}

void GrabAction::setSucceeded ()
{
	handling_msgs::ObjectGrabbed objectGrab;
	objectGrab.isGrabbed = true;
	objectGrabbedPub.publish (objectGrab);
	plan->setGrabbedObject (objectID.front ());

	ROS_INFO_STREAM("Grab pipeline has been successful :) ");
	handling_msgs::GoapMsgActionResult result;
	result.result.status = "Succeeded";
	server->setSucceeded (result.result, "$activeObject.isCarried:True;$robot.hasObj:$activeObject");
	paramList->setExecutionMode (ArmControlParameterList::ExecutionMode::Joystick);
	handling_msgs::ToggleGUIExecuteCancelBtn toggleReq;
	toggleReq.request.enableExecuteBtn = true;
	client.call (toggleReq);
}

void GrabAction::setPreempted ()
{
	goalPreemptCB ();
}

bool GrabAction::isActive ()
{
	return server->isActive ();
}
