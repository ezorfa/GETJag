/*
 * InspectAction.cpp
 *
 *  Created on: Feb 7, 2019
 *      Author: mohammed
 *
 *  This class implements a pipeline/ sequence of actions to be performed in order to inspect an object.
 *  Hosts a server to which goal can be sent using an action client.
 */

#include <utility>

#include <yaml-cpp/yaml.h>
#include <std_msgs/Bool.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <arm_planning/InspectAction.h>
#include <handling_msgs/ObjectGrabbed.h>

InspectAction::InspectAction (ros::NodeHandle& nh, std::shared_ptr<ArmControlParameterList>& paramList, std::shared_ptr<Executor>& exe, std::shared_ptr<Planning>& plan)
		: nh (nh), exe (exe), plan (plan), paramList (paramList), spinner (1, &callbackQueue)
{
	nhPrivate.setCallbackQueue (&callbackQueue);
	spinner.start ();
	staticParams = paramList->getStaticParams ();
	objectGrabbedPub = nh.advertise<handling_msgs::ObjectGrabbed> (staticParams.objectGrabbed, 10, true);
	poseGoalClient = std::make_unique<actionlib::SimpleActionClient<handling_msgs::GoapMsgAction>> (staticParams.goalActionName, true);
	gripperGoalClient = std::make_unique<actionlib::SimpleActionClient<handling_msgs::GoapMsgAction>> (staticParams.gripperActionName, true);
	client = nhPrivate.serviceClient<handling_msgs::ToggleGUIExecuteCancelBtn> (staticParams.GUIToggleExeCancel);
	server = std::make_unique<InspectServer> (nhPrivate, staticParams.inspectActionName, false);
	server->registerGoalCallback (boost::bind (&InspectAction::goalCB, this));
	server->registerPreemptCallback (boost::bind (&InspectAction::goalPreemptCB, this));
	server->start ();
	listener = std::make_shared<tf::TransformListener> ();

	ROS_INFO_STREAM("Inspect Action Pipeline has been started successfully!");
}

InspectAction::~InspectAction ()
{
	server->shutdown ();
}

void InspectAction::doTransform (const geometry_msgs::Pose in, geometry_msgs::Pose &out, const std::string target, const std::string base)
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

void InspectAction::goalCB ()
{
	ROS_INFO_STREAM("Command to Inspect pipeline is received");

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
	
    //******[[ !!! Below lines need to be uncommented if doing integrated tests with all other groups ]]******
	//Transforming from odom frame into base_link frame
	//geometry_msgs::Pose before = pose;
	//doTransform (before, pose, "/GETjag/base_link", "/GETjag/odom");

	//ROS_INFO_STREAM("Transformed Pose: ");
	//ROS_INFO_STREAM(pose);

	std::string object = yaml["$activeObject.id"].as<std::string> ();
	objectID.clear ();
	objectID.push_back (object);
	inspects = paramList->getAllInspectionParams (objectID.front ());

	if (not (yaml["$activeObject.which"].as<std::string> () == "None"))
	{
		ROS_INFO_STREAM("Planning for only selected Inspect points.");
		std::string which = yaml["$activeObject.which"].as<std::string> ();
		std::map<std::string, std::vector<double>> temp;
		temp[which] = inspects[which];
		inspects.clear ();
		inspects = temp;
	}
	else
	{
		ROS_INFO_STREAM("Planning for all Inspects.");
	}

	paramList->setExecutionMode (ArmControlParameterList::ExecutionMode::Automatic);
	std::string meshPath = paramList->getMeshPath (objectID.front ());

	/***************************** Add the object as a collision object ****************************************/

	plan->addMeshCollisionObjects (objectID, meshPath, pose);

	ros::Duration (2).sleep ();
	/***************************** Selection of best feasable grasps *******************************************/

	tf2::Quaternion orientation;

	bool isPlanningSuccessful = false;

	nhPrivate.setParam ("currentActionStatus", "starting planning for the action...");
	setFeedback (5);

	for (auto const& element : inspects)
	{
		moveit_msgs::RobotTrajectory traj;
		pose.position.x = (double)pose.position.x + (double)element.second[3] + (double)element.second[6] + (double)element.second[9];
		pose.position.y = (double)pose.position.y + (double)element.second[4] + (double)element.second[7] + (double)element.second[10];
		pose.position.z = (double)pose.position.z + (double)element.second[5] + (double)element.second[8] + (double)element.second[11];

		orientation.setRPY (-element.second[0], element.second[1], element.second[2]);
		pose.orientation = tf2::toMsg (orientation);
		pose.orientation.x = -pose.orientation.x;

		if (isCancelRequested ())
			return;
		if (plan->plan (pose, traj, true))
		{
			isPlanningSuccessful = true;
			break;
		}
		else
			continue;
	}

	nhPrivate.setParam ("currentActionStatus", "Planning is completed...");
	setFeedback (10);

	handling_msgs::GoapMsgAction goal;

	if (not isPlanningSuccessful)
	{
		handling_msgs::GoapMsgActionResult result;
		result.result.status = "Succeeded";

		YAML::Emitter r;
		r << YAML::BeginMap;
		r << YAML::Key << "$activeObject.inspected";
		r << YAML::Value << "False";
		r << YAML::EndMap;

		result.result.result = r.c_str ();
		server->setSucceeded (result.result);
//		goalPreemptCB ();
		return;
	}
	else // START PIPELINE EXECUTION !
	{
		nhPrivate.setParam ("currentActionStatus", "Arm is moving near object...");
		setFeedback (65);
		std::string goalString = convertGoalToGoapMsg (pose);
		goal.action_goal.goal.input = goalString;

		/************************************* Move near object *********************************************/

		poseGoalClient->sendGoal (goal.action_goal.goal);
		bool result = poseGoalClient->waitForResult ();
		if (result)
		{
			if (poseGoalClient->getState ().state_ == actionlib::SimpleClientGoalState::SUCCEEDED)
			{
				nhPrivate.setParam ("currentActionStatus", "Action succeeded...Arm is in the inspect position !!!");
				setFeedback (100);
				setSucceeded ();
			}
		}
		else
		{
			plan->dettachObjectFromTool (objectID.front ());

			handling_msgs::GoapMsgActionResult result;
			result.result.status = "Succeeded";

			YAML::Emitter r;
			r << YAML::BeginMap;
			r << YAML::Key << "$activeObject.inspected";
			r << YAML::Value << "False";
			r << YAML::EndMap;

			result.result.result = r.c_str ();
			server->setSucceeded (result.result);
//			goalPreemptCB ();
			return;
		}
	}
}

bool InspectAction::isCancelRequested ()
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

std::string InspectAction::convertGoalToGoapMsg (const geometry_msgs::Pose& pose) const
{
	std::string goapMsg = "{$activeObject.atPos : [" + std::to_string (pose.position.x) + "," + std::to_string (pose.position.y) + "," + std::to_string (pose.position.z) + "," + std::to_string (pose.orientation.x) + "," + std::to_string (pose.orientation.y) + "," + std::to_string (pose.orientation.z) + "," + std::to_string (pose.orientation.w) + "]}";

	return goapMsg;
}

void InspectAction::goalPreemptCB ()
{
	nhPrivate.setParam ("currentActionStatus", "Preempted...Goal unsuccessful!");
	setFeedback (-1);

	handling_msgs::ToggleGUIExecuteCancelBtn toggleReq;
	toggleReq.request.enableExecuteBtn = true;
	client.call (toggleReq);

	handling_msgs::GoapMsgActionResult result;
	result.result.status = "incomplete";
	server->setPreempted (result.result, "planning failed!");
	cleanUp ();
}

void InspectAction::cleanUp ()
{
	ROS_INFO_STREAM("Inspect pipeline is not successful :( ");
	if (not objectID.empty ())
	{
		plan->removeCollisionObjects (objectID);
	}
	paramList->setExecutionMode (ArmControlParameterList::ExecutionMode::Joystick);
}

void InspectAction::setFeedback (double fb)
{
	handling_msgs::GoapMsgActionFeedback feedback;
	feedback.feedback.progress = fb;
	server->publishFeedback (feedback.feedback);
}

void InspectAction::setSucceeded ()
{
	handling_msgs::ToggleGUIExecuteCancelBtn toggleReq;
	toggleReq.request.enableExecuteBtn = true;
	client.call (toggleReq);

	ROS_INFO_STREAM("Inspect pipeline has been successful :) ");
	plan->removeCollisionObjects (objectID);

	handling_msgs::GoapMsgActionResult result;
	result.result.status = "Succeeded";

	YAML::Emitter r;
	r << YAML::BeginMap;
	r << YAML::Key << "$activeObject.inspected";
	r << YAML::Value << "True";
	r << YAML::EndMap;

	result.result.result = r.c_str ();
	server->setSucceeded (result.result);

	paramList->setExecutionMode (ArmControlParameterList::ExecutionMode::Joystick);
}

void InspectAction::setPreempted ()
{
	goalPreemptCB ();
}

bool InspectAction::isActive ()
{
	return server->isActive ();
}
