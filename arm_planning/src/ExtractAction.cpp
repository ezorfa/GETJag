/*
 * ExtractAction.cpp
 *
 *  Created on: Feb 7, 2019
 *      Author: Mohammed Afroze
 */

#include <utility>

#include <yaml-cpp/yaml.h>
#include <std_msgs/Bool.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <arm_planning/ExtractAction.h>
#include <handling_msgs/ObjectGrabbed.h>

ExtractAction::ExtractAction (ros::NodeHandle& nh, std::shared_ptr<ArmControlParameterList>& paramList, std::shared_ptr<Executor>& exe, std::shared_ptr<Planning>& plan)
		: nh (nh), exe (exe), plan (plan), paramList (paramList), spinner (1, &callbackQueue)
{
	nhPrivate.setCallbackQueue (&callbackQueue);
	spinner.start ();

	objectGrabbedPub = nhPrivate.advertise<handling_msgs::ObjectGrabbed> ("/dexterity/object_grabbed_extract", 10, true);
	staticParams = paramList->getStaticParams ();
	poseGoalClient = std::make_unique<actionlib::SimpleActionClient<handling_msgs::GoapMsgAction>> (staticParams.goalActionName, true);
	gripperGoalClient = std::make_unique<actionlib::SimpleActionClient<handling_msgs::GoapMsgAction>> (staticParams.gripperActionName, true);
	client = nhPrivate.serviceClient<handling_msgs::ToggleGUIExecuteCancelBtn> (staticParams.GUIToggleExeCancel);
	server = std::make_unique<ExtractServer> (nhPrivate, staticParams.extractActionName, false);
	server->registerGoalCallback (boost::bind (&ExtractAction::goalCB, this));
	server->registerPreemptCallback (boost::bind (&ExtractAction::goalPreemptCB, this));
	server->start ();
	listener = std::make_shared<tf::TransformListener> ();

	ROS_INFO_STREAM("Extract Pipeline has been started successfully!");
}

ExtractAction::~ExtractAction ()
{
	server->shutdown ();
}

void ExtractAction::doTransform (const geometry_msgs::Pose in, geometry_msgs::Pose &out, const std::string target, const std::string base)
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

void ExtractAction::goalCB ()
{
	ROS_INFO_STREAM("Command to Extract pipeline is received");
	bool continuePipeline = false;

	exe->setServer (this);

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
	extractPoints = paramList->getAllExtractionParams (objectID.front ());

	if (not yaml["$activeObject.which"].IsNull ())
	{
		ROS_INFO_STREAM("Planning for only selected extractPoints.");
		std::string which = yaml["$activeObject.which"].as<std::string> ();
		std::map<std::string, std::vector<double>> temp;
		temp[which] = extractPoints[which];
		extractPoints.clear ();
		extractPoints = temp;
	}
	else
	{
		ROS_INFO_STREAM("Planning for all Extraction points.");
	}

	paramList->setExecutionMode (ArmControlParameterList::ExecutionMode::Automatic);

	std::string meshPath = paramList->getMeshPath (objectID.front ());

	/***************************** Add the object as a collision object ************************************/

	plan->addMeshCollisionObjects (objectID, meshPath, pose);
	ros::Duration (2).sleep ();

	/***************************** Selection of best feasable extractionPoints *******************************/

	tf2::Quaternion orientation;
	std::string WhichPoint;
	bool isPlanningSuccessful = false;

	nhPrivate.setParam ("currentActionStatus", "starting planning for the action...");
	setFeedback (5);

	for (auto const& element : extractPoints)
	{
		geometry_msgs::Pose tempPose = pose;
		moveit_msgs::RobotTrajectory traj;
		tempPose.position.x = (double)tempPose.position.x + (double)element.second[3] + (double)element.second[6] + (double)element.second[9];
		tempPose.position.y = (double)tempPose.position.y + (double)element.second[4] + (double)element.second[7] + (double)element.second[10];
		tempPose.position.z = (double)tempPose.position.z + (double)element.second[5] + (double)element.second[8] + (double)element.second[11];
		orientation.setRPY (-element.second[0], element.second[1], element.second[2]);

		tempPose.orientation = tf2::toMsg (orientation);
		tempPose.orientation.x = -tempPose.orientation.x;

		ROS_INFO_STREAM("which inspect?");
		ROS_INFO_STREAM(element.first);
		ROS_INFO_STREAM(tempPose);

		auto tempPose1 = tempPose;
		// This flag is used as an indicator whether to stop the current planning/ execution.
		bool continuePlanning = false;
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
			//ros::Duration (2).sleep ();
			//tempPose1.position.x = (double)tempPose1.position.x - (double)element.second[3] - (double)element.second[12];
			//tempPose1.position.y = (double)tempPose1.position.y - (double)element.second[4] - (double)element.second[13];
			//tempPose1.position.z = (double)tempPose1.position.z - (double)element.second[5] - (double)element.second[14];
			// plan to approach object.
			//if (plan->plan (tempPose1, traj, true))
			//{
				WhichPoint = element.first;
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
			pose.position.x = pose.position.x - extractPoints[WhichPoint][3] - extractPoints[WhichPoint][12];
			pose.position.y = pose.position.y - extractPoints[WhichPoint][4] - extractPoints[WhichPoint][13];
			pose.position.z = pose.position.z - extractPoints[WhichPoint][5] - extractPoints[WhichPoint][14];

			goal.action_goal.goal.input = convertGoalToGoapMsg (pose);
			poseGoalClient->sendGoal (goal.action_goal.goal);
			bool result = poseGoalClient->waitForResult ();
			if (result)
			{
				if (poseGoalClient->getState ().state_ == actionlib::SimpleClientGoalState::SUCCEEDED)
				{
					nhPrivate.setParam ("currentActionStatus", "Approaching object...");
					setFeedback (70);
					continuePipeline = true;
				}
			}
		}
		if (isCancelRequested ())
			return;

		ROS_INFO_STREAM("Pose before extraction");
		ROS_INFO_STREAM(pose);

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
					nhPrivate.setParam ("currentActionStatus", "Closing gripper...Extracting...");
					setFeedback (80);
					continuePipeline = true;
					handling_msgs::ObjectGrabbed objectGrab;
					objectGrab.isGrabbed = true;
					objectGrabbedPub.publish (objectGrab);
				}
			}

			if (isCancelRequested ())
				return;

			/*************************************** Extraction ************************************************/
			if (continuePipeline)
			{
				continuePipeline = false;
				pose.position.x = pose.position.x + extractPoints[WhichPoint][15];
				pose.position.y = pose.position.y + extractPoints[WhichPoint][16];
				pose.position.z = pose.position.z + extractPoints[WhichPoint][17];

				ROS_INFO_STREAM("Pose after extraction");
				ROS_INFO_STREAM(pose);

				goal.action_goal.goal.input = convertGoalToGoapMsg (pose);
				poseGoalClient->sendGoal (goal.action_goal.goal);
				bool result = poseGoalClient->waitForResult ();
				if (result)
				{
					if (poseGoalClient->getState ().state_ == actionlib::SimpleClientGoalState::SUCCEEDED)
					{
						nhPrivate.setParam ("currentActionStatus", "Action completed !!!");
						setFeedback (100);
						setSucceeded ();
					}
					else
					{
						handling_msgs::GoapMsgActionResult result;
						result.result.status = "Succeeded";
						server->setSucceeded (result.result, "$activeObject.isCarried:False;$robot.hasObj:None");
						//			goalPreemptCB ();
					}

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

bool ExtractAction::isCancelRequested ()
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

std::string ExtractAction::convertGoalToGoapMsg (const geometry_msgs::Pose& pose) const
{
	std::string goapMsg = "{$activeObject.atPos : [" + std::to_string (pose.position.x) + "," + std::to_string (pose.position.y) + "," + std::to_string (pose.position.z) + "," + std::to_string (pose.orientation.x) + "," + std::to_string (pose.orientation.y) + "," + std::to_string (pose.orientation.z) + "," + std::to_string (pose.orientation.w) + "]}";

	return goapMsg;
}

void ExtractAction::cleanUp ()
{
	ROS_INFO_STREAM("Extract pipeline is not successful :( ");
	if (isObjectAttacedToTool)
		plan->dettachObjectFromTool (objectID.front ());
	if (not objectID.empty ())
	{
		plan->removeCollisionObjects (objectID);
		ros::Duration (2).sleep ();
	}
	paramList->setExecutionMode (ArmControlParameterList::ExecutionMode::Joystick);
}

void ExtractAction::goalPreemptCB ()
{
	nhPrivate.setParam ("currentActionStatus", "Preempted...Goal unsuccessful!");
	setFeedback (-1);

	handling_msgs::ToggleGUIExecuteCancelBtn toggleReq;
	toggleReq.request.enableExecuteBtn = true;
	client.call (toggleReq);

	handling_msgs::GoapMsgActionResult result;
	result.result.status = "incomplete";
	cleanUp ();
	server->setPreempted (result.result, "planning failed!");
}

void ExtractAction::setFeedback (double fb)
{
	handling_msgs::GoapMsgActionFeedback feedback;
	feedback.feedback.progress = fb;
	server->publishFeedback (feedback.feedback);
}

void ExtractAction::setSucceeded ()
{

	plan->setGrabbedObject (objectID.front ());

	handling_msgs::ToggleGUIExecuteCancelBtn toggleReq;
	toggleReq.request.enableExecuteBtn = true;
	client.call (toggleReq);

	ROS_INFO_STREAM("Extract pipeline is successful :) ");

	handling_msgs::GoapMsgActionResult result;
	result.result.result = "complete";
	server->setSucceeded (result.result, "goal reached");
	paramList->setExecutionMode (ArmControlParameterList::ExecutionMode::Joystick);
}

void ExtractAction::setPreempted ()
{
	goalPreemptCB ();
}

bool ExtractAction::isActive ()
{
	return server->isActive ();
}
