/*
 * ArmControlParameterList.cpp
 *
 *  Created on: Mar 22, 2019
 *      Author: Mohammed Afroze
 */

#include <cstdlib>
#include <exception>
#include <iostream>
#include <map>
#include <string>
#include <vector>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>

#include <arm_planning/ArmControlParameterList.h>
#include <handling_msgs/GetObjectHandlingParam.h>

ArmControlParameterList::ArmControlParameterList (ros::NodeHandle& nh)
		: nh (nh)
{
	joyMode = JoyControlMode::PLANNING;
	mode = ExecutionMode::Automatic;
	gripperMode = GripperMode::CLOSE;
//	paramClient = nh.serviceClient<beginner_tutorials::AddTwoInts>("dexterity/get_action_parameters");
}

/* This function gets all the controllers contained in the yaml file*/
std::map<std::string, std::string> ArmControlParameterList::getControllers ()
{
	if (not controllers.empty ())
		return controllers;

	XmlRpc::XmlRpcValue controller_names;

	getParam ("/init/controllers", controller_names);
	auto joints = getAllJoints ();

	int i = 0;
	for (auto joint : joints)
	{
		XmlRpc::XmlRpcValue &controller_value = controller_names[i];
		controllers[joint] = ((std::string)controller_value);
		i++;
	}
	return controllers;
}

/* This function gets all the joints contained in the yaml file*/
std::vector<std::string> ArmControlParameterList::getAllJoints ()
{
	if (not jointNames.empty ())
		return jointNames;

	XmlRpc::XmlRpcValue joint_names;

	getParam ("/init/joints", joint_names);

	for (int i = 0; i < joint_names.size (); ++i)
	{
		XmlRpc::XmlRpcValue &name_value = joint_names[i];
		jointNames.push_back ((std::string)name_value);
	}

	return jointNames;
}

/* This function gets all the links that are expected to touch the object for performing actions like grab, extraction*/
std::vector<std::string> ArmControlParameterList::getTouchLinks ()
{
	if (not touchLinks.empty ())
		return touchLinks;

	XmlRpc::XmlRpcValue links;

	getParam ("/init/touch_links", links);

	for (int i = 0; i < links.size (); ++i)
	{
		XmlRpc::XmlRpcValue &value = links[i];
		touchLinks.push_back ((std::string)value);
		ROS_INFO_STREAM(touchLinks[i]);
	}

	return touchLinks;
}

/* This function gets all the joints that are related to arm's wrist*/
std::vector<std::string> ArmControlParameterList::getWristJoints ()
{
	if (not endEffJoints.empty ())
		return endEffJoints;

	auto temp = getAllJoints ();

	for (int i = 3; i < temp.size (); ++i)
	{
		endEffJoints.push_back (temp[i]);
	}

	return endEffJoints;
}

std::map<std::string, double> ArmControlParameterList::getJointSpeedLimits ()
{
	std::map<std::string, double> map;
	double temp;
	auto joints = getAllJoints ();
	for (auto& joint : joints)
	{
		getParam ("/robot_description_planning/joint_limits/" + joint + "/max_velocity", temp);
		map[joint] = temp;
	}
	return map;
}

JoystickMappings ArmControlParameterList::getJoyMappings ()
{
	getParam ("/init/joy_mappings/AXIS_X", joyMappings.AXIS_X);
	getParam ("/init/joy_mappings/AXIS_Y", joyMappings.AXIS_Y);
	getParam ("/init/joy_mappings/AXIS_Z", joyMappings.AXIS_Z);
	getParam ("/init/joy_mappings/BTN_MANUAL", joyMappings.BTN_MANUAL);
	getParam ("/init/joy_mappings/BTN_PLANNING", joyMappings.BTN_PLANNING);
	getParam ("/init/joy_mappings/BTN_WRIST_RESET", joyMappings.BTN_WRIST_RESET);
	getParam ("/init/joy_mappings/BTN_RESET", joyMappings.BTN_RESET);
	getParam ("/init/joy_mappings/AXIS_PITCH", joyMappings.AXIS_PITCH);
	getParam ("/init/joy_mappings/AXIS_ROLL", joyMappings.AXIS_ROLL);
	getParam ("/init/joy_mappings/AXIS_YAW", joyMappings.AXIS_YAW);
	getParam ("/init/joy_mappings/AXIS_SHOULDER", joyMappings.AXIS_SHOULDER);

	getParam ("/init/joy_mappings/AXIS_BASE", joyMappings.AXIS_BASE);
	getParam ("/init/joy_mappings/AXIS_ELBOW", joyMappings.AXIS_ELBOW);
	getParam ("/init/joy_mappings/BTN_G_OPEN", joyMappings.BTN_G_OPEN);
	getParam ("/init/joy_mappings/BTN_G_CLOSE", joyMappings.BTN_G_CLOSE);
	getParam ("/init/joy_mappings/BTN_AUTO", joyMappings.BTN_AUTO);
	getParam ("/init/joy_mappings/BTN_BASELINK", joyMappings.BTN_BASELINK);
	getParam ("/init/joy_mappings/BTN_EEF", joyMappings.BTN_EEF);

	return joyMappings;
}

Parameters ArmControlParameterList::getStaticParams ()
{
	getParam ("/init/robot_namespace", parameters.robotNamespace);
	getParam ("/init/topic_names/controller_speed", parameters.speedTopic);
	getParam ("/init/gripper_controller", parameters.gripperController);
	getParam ("/init/gripper", parameters.gripper);

	getParam ("/init/action_names/go_home", parameters.goHomeActionName);
	getParam ("/init/action_names/go_to_pose", parameters.goalActionName);
	getParam ("/init/action_names/grab", parameters.grabActionName);
	getParam ("/init/action_names/gripper", parameters.gripperActionName);
	getParam ("/init/action_names/extract", parameters.extractActionName);
	getParam ("/init/action_names/inspect", parameters.inspectActionName);
	getParam ("/init/action_names/drop", parameters.dropActionName);

	getParam ("/init/reference_frames/base_link", parameters.baseLinkRefFrame);
	getParam ("/init/reference_frames/end_effector_link", parameters.endEffectorRefFrame);

	getParam ("/init/topic_names/controller_command", parameters.commandTopic);
	getParam ("/init/topic_names/controller_speed", parameters.stateTopic);
	getParam ("/init/topic_names/controller_state", parameters.stateTopic);
	getParam ("/init/topic_names/joint_states", parameters.jointStatesTopic);
	getParam ("/init/topic_names/object_grabbed_grasp", parameters.objectGrabbedGrasp);
	getParam ("/init/topic_names/object_grabbed_extract", parameters.objectGrabbedExtract);
	getParam ("/init/topic_names/gui_toggle_exe_cancel", parameters.GUIToggleExeCancel);
	getParam ("/init/topic_names/wrist_reset", parameters.wristReset);
	getParam ("/init/topic_names/reset", parameters.reset);

	getParam ("/init/constants/maxScalingFactorJoyPlanMode", parameters.maxScalingFactorJoyPlanMode);
	getParam ("/init/constants/octomapRefreshCount", parameters.octomapRefreshCount);
	getParam ("/init/constants/executionTimeOutCount", parameters.executionTimeOutCount);
	getParam ("/init/constants/joyModeErrorMargin", parameters.joyModeErrorMargin);
	getParam ("/init/constants/autoModeErrorMargin", parameters.autoModeErrorMargin);
	getParam ("/init/constants/lowerSpeedLimit", parameters.lowerSpeedLimit);
	getParam ("/init/constants/joyMaxVelocityScalingFactor", parameters.joyMaxVelocityScalingFactor);
	getParam ("/init/constants/grabTimeOutCount", parameters.grabTimeOutCount);
	getParam ("/init/constants/moveitMaxVelocityScalingFactor", parameters.moveitMaxVelocityScalingFactor);
	getParam ("/init/constants/moveitMaxPlanningAttempts", parameters.moveitMaxPlanningAttempts);
	getParam ("/init/constants/moveitGoalTolerance", parameters.moveitGoalTolerance);
	getParam ("/init/constants/moveitEEFStep", parameters.moveitEEFStep);
	getParam ("/init/constraints/joyPlanningXLowerLimit", parameters.joyPlanningXLowerLimit);
	getParam ("/init/constraints/joyPlanningXHigherLImit", parameters.joyPlanningXHigherLImit);
	getParam ("/init/constants/gripperOpenAngle", parameters.gripperOpenAngle);
	getParam ("/init/constants/manualJointSpeeds", parameters.manualJointSpeeds);
	getParam ("/init/constants/maxShoulderScalingFactorManualMode", parameters.maxShoulderScalingFactorManualMode);
	getParam ("/init/constants/maxElbowScalingFactorManualMode", parameters.maxElbowScalingFactorManualMode);
	getParam ("/init/constants/maxBaseScalingFactorManualMode", parameters.maxBaseScalingFactorManualMode);
	getParam ("/init/constants/maxPitchScalingFactorManualMode", parameters.maxPitchScalingFactorManualMode);
	getParam ("/init/constants/maxRollScalingFactorManualMode", parameters.maxRollScalingFactorManualMode);
	getParam ("/init/constants/maxYawScalingFactorManualMode", parameters.maxYawScalingFactorManualMode);
	getParam ("/init/constants/maxGrabLoad", parameters.maxGrabLoad);

	return parameters;
}

/* This function gets the location of the object's mesh contained in the yaml file*/
std::string ArmControlParameterList::getMeshPath (const std::string object)
{
	std::string path;
	getParam ("/init/object_meshes_path/" + object, path);
	return path;
}

std::map<std::string, std::vector<double>> ArmControlParameterList::getAllExtractionParams (const std::string& objectID)
{
	std::map<std::string, std::vector<double>> extractPoses;
	loadParamsFromFile ("extraction.yaml", extractPoses, objectID);
	//~ getParamsFromDatabase("extract",extractPoses,objectID);
	if (extractPoses.empty ())
		ROS_ERROR("No extract points specified for the object");

	return extractPoses;
}

std::map<std::string, std::vector<double>> ArmControlParameterList::getAllInspectionParams (const std::string& objectID)
{
	std::map<std::string, std::vector<double>> inspectPoses;
	loadParamsFromFile ("inspection.yaml", inspectPoses, objectID); 
	//~ getParamsFromDatabase("inspect",inspectPoses,objectID);

	if (inspectPoses.empty ())
		ROS_ERROR("No Inspect points specified for the object");

	return inspectPoses;
}

std::map<std::string, std::vector<double>> ArmControlParameterList::getAllGrasps (const std::string& objectID)
{
	std::map<std::string, std::vector<double>> grasps;
	loadParamsFromFile ("grasps.yaml", grasps, objectID);
	//~ getParamsFromDatabase("grasp",grasps,objectID);

	if (grasps.empty ())
		ROS_ERROR("No grasps specified for the object");



	return grasps;
}

void ArmControlParameterList::loadParamsFromFile (const std::string yamlFileName, std::map<std::string, std::vector<double>>& container, const std::string& objectID)
{
	container.clear ();
	std::string path = ros::package::getPath ("arm_planning") + "/config/" + yamlFileName;
	YAML::Node root;
	try
	{
		root = YAML::LoadFile (path);
	}
	catch (const std::exception& e)
	{
		std::cout << e.what () << "\n";
		return;
	}
	int i = 0;
	std::vector<double> temp;
	for (YAML::const_iterator it = root.begin (); it != root.end (); ++it)
	{
		if (it->first.as<std::string> () == objectID)
		{
			for (YAML::const_iterator it2 = it->second.begin (); it2 != it->second.end (); ++it2)
			{
				temp.clear ();

				for (YAML::const_iterator it3 = it2->second.begin (); it3 != it2->second.end (); ++it3)
				{
					temp.push_back (it3->as<double> ());
				}
				container[it2->first.as<std::string> ()] = temp;

				i++;
			}
		}
	}
}

void ArmControlParameterList::getParamsFromDatabase (const std::string actionName, std::map<std::string, std::vector<double>>& container, const std::string& objectID)
{
	ros::ServiceClient client = nh.serviceClient<handling_msgs::GetObjectHandlingParam> ("detection/database/getActionPoints");
	handling_msgs::GetObjectHandlingParam reqMsg;
	reqMsg.request.actionType = "actionName";
	reqMsg.request.modelName = objectID;
	client.call (reqMsg);

	auto actionNames = reqMsg.response.actionName;
	auto actionPoints = reqMsg.response.actionPoint;

	if (not actionPoints.empty ())
	{
		std::vector<double> points;
		int i = 0;
		for (auto pt : actionPoints)
		{
			split2double (pt, ',', std::back_inserter (points));
			container[actionNames.at (i)] = points;
			points.clear ();
			i++;
		}
	}

}

void ArmControlParameterList::setGripperMode (GripperMode mode)
{
	int temp = mode;
	nh.setParam ("/init/constants/currentGripperStatus", temp);
}

int ArmControlParameterList::getGripperMode ()
{
	int temp;
	getParam ("/init/constants/currentGripperStatus", temp);

	return temp;
}

void ArmControlParameterList::setExecutionMode (ArmControlParameterList::ExecutionMode mode)
{
	std::string temp;
	if (mode == ArmControlParameterList::ExecutionMode::Automatic)
		temp = "Automatic";
	else
		temp = "Joystick";
	try
	{
		nh.setParam ("/init/current_execution_mode", temp);
	}
	catch (ros::Exception& e)
	{
		std::cout << "Exception : " << e.what () << std::endl;
	}
}
ArmControlParameterList::ExecutionMode ArmControlParameterList::getExecutionMode ()
{
	std::string temp;
	getParam ("/init/current_execution_mode", temp);
	if (temp == "Automatic")
		mode = ArmControlParameterList::ExecutionMode::Automatic;
	else
		mode = ArmControlParameterList::ExecutionMode::Joystick;

	return mode;
}

void ArmControlParameterList::setJoystickControlMode (ArmControlParameterList::JoyControlMode mode)
{
	std::string temp;
	if (mode == ArmControlParameterList::JoyControlMode::MANUAL)
		temp = "Manual";
	else
		temp = "Planning";

	nh.setParam ("/init/current_joy_mode", temp);
}

ArmControlParameterList::JoyControlMode ArmControlParameterList::getJoystickControlMode ()
{
	std::string temp;
	getParam ("/init/current_joy_mode", temp);

	if (temp == "Manual")
		joyMode = ArmControlParameterList::JoyControlMode::MANUAL;
	else
		joyMode = ArmControlParameterList::JoyControlMode::PLANNING;

	return joyMode;
}

void ArmControlParameterList::setRefFrame (std::string frame)
{
	nh.setParam ("/init/reference_frames/current_ref_frame", frame);
}
std::string ArmControlParameterList::getRefFrame ()
{
	std::string temp;
	getParam ("/init/reference_frames/current_ref_frame", temp);

	return temp;
}

template<typename T>
void ArmControlParameterList::getParam (const std::string& key, T& value)
{
	ros::NodeHandle pn ("~");
	if (not pn.getParam (key, value))
	{
		ROS_ERROR("parameter %s not set", key.c_str ());
		exit (1);
	}
}

template<typename Out>
void ArmControlParameterList::split2double (const std::string &s, char delim, Out result)
{
	std::stringstream ss (s);
	std::string item;
	while (std::getline (ss, item, delim))
	{
		std::cout << std::stod (item) << "\n";
		*(result++) = std::stod (item);
	}
}
