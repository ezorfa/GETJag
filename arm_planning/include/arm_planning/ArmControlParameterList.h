/*
 * ArmControlParameterList.h
 *
 *  Created on: Mar 22, 2019
 *      Author: Mohammed Afroze
 *
 * ArmControlParameterList class forms as an intermediatary for all other nodes and contains
 * getter and setter methods to set/get values from/on the param server and yaml files.
 *
 */

#ifndef ROS_PG_WS_2018_19_ARM_PLANNING_INCLUDE_ARM_PLANNING_ARMCONTROLPARAMETERLIST_H_
#define ROS_PG_WS_2018_19_ARM_PLANNING_INCLUDE_ARM_PLANNING_ARMCONTROLPARAMETERLIST_H_

#include <ros/ros.h>
#include <dynamixel_msgs/JointState.h>

/* Parameters contains all the necessary parameters used to setup all nodes */
struct Parameters
{
		std::string gripper;
		std::string gripperController;
		std::string robotNamespace;
		std::string speedTopic;
		std::string commandTopic;
		std::string stateTopic;
		std::string jointStatesTopic;
		std::string grabActionName;
		std::string goalActionName;
		std::string gripperActionName;
		std::string goHomeActionName;
		std::string inspectActionName;
		std::string extractActionName;
		std::string baseLinkRefFrame;
		std::string endEffectorRefFrame;
		std::string objectGrabbed;
		std::string dropActionName;
		std::string wristReset;
		std::string reset;
		std::string objectGrabbedGrasp;
		std::string objectGrabbedExtract;
		std::string GUIToggleExeCancel;
		double maxGrabLoad;
		double octomapRefreshCount;
		double executionTimeOutCount;
		double errorMargin;
		double lowerSpeedLimit;
		double joyMaxVelocityScalingFactor;
		double grabTimeOutCount;
		double moveitMaxVelocityScalingFactor;
		double moveitMaxPlanningAttempts;
		double moveitGoalTolerance;
		double moveitEEFStep;
		double joyPlanningXLowerLimit;
		double joyPlanningXHigherLImit;
		double gripperOpenAngle;
		double manualJointSpeeds;
		double joyModeErrorMargin;
		double autoModeErrorMargin;
		double maxScalingFactorJoyPlanMode;
		double maxShoulderScalingFactorManualMode;
		double maxElbowScalingFactorManualMode;
		double maxBaseScalingFactorManualMode;
		double maxPitchScalingFactorManualMode;
		double maxRollScalingFactorManualMode;
		double maxYawScalingFactorManualMode;
};

struct JoystickMappings
{
		int AXIS_X;
		int AXIS_Y;
		int AXIS_Z;
		int BTN_MANUAL;
		int BTN_PLANNING;
		int BTN_WRIST_RESET;
		int BTN_RESET;
		int AXIS_PITCH;
		int AXIS_ROLL;
		int AXIS_YAW;
		int AXIS_SHOULDER;
		int AXIS_BASE;
		int AXIS_ELBOW;
		int BTN_G_OPEN;
		int BTN_G_CLOSE;
		int BTN_AUTO;
		int BTN_BASELINK;
		int BTN_EEF;
};

class ArmControlParameterList
{
	public:
		ArmControlParameterList (ros::NodeHandle& nh);
		~ArmControlParameterList () = default;

		enum ExecutionMode
		{
			Joystick, Automatic
		};
		enum JoyControlMode
		{
			PLANNING, MANUAL
		};
		enum GripperMode
		{
			CLOSE = 0, OPEN
		};

		void setRefFrame (std::string frame);
		void setExecutionMode (ExecutionMode mode);
		void setGripperMode (GripperMode mode);
		void setJoystickControlMode (JoyControlMode mode);
		std::string getRefFrame ();
		std::string getMeshPath (const std::string object);
		int getGripperMode ();
		std::vector<std::string> getAllJoints ();
		std::vector<std::string> getWristJoints ();
		std::vector<std::string> getTouchLinks ();
		std::map<std::string, double> getJointSpeedLimits ();
		std::map<std::string, std::string> getControllers ();
		std::map<std::string, std::vector<double>> getAllGrasps (const std::string& objectID);
		std::map<std::string, std::vector<double>> getAllExtractionParams (const std::string& objectID);
		std::map<std::string, std::vector<double>> getAllInspectionParams (const std::string& objectID);
		Parameters getStaticParams ();
		JoystickMappings getJoyMappings ();
		ArmControlParameterList::JoyControlMode getJoystickControlMode ();
		ArmControlParameterList::ExecutionMode getExecutionMode ();

		using Ptr = std::shared_ptr<ArmControlParameterList>;

	private:
		template<typename T>
		void getParam (const std::string& key, T& value);
		void loadParamsFromFile (const std::string yamlFileName, std::map<std::string, std::vector<double>>& container, const std::string& objectID);
		void getParamsFromDatabase(const std::string actionName, std::map<std::string, std::vector<double>>& container, const std::string& objectID);
		template<typename Out>
		void split2double(const std::string &s, char delim, Out result);
		ros::NodeHandle nh;
		ros::ServiceClient paramClient;

		Parameters parameters;
		JoystickMappings joyMappings;
		ExecutionMode mode;
		JoyControlMode joyMode;
		GripperMode gripperMode;

		std::string refFrame;
		std::vector<std::string> endEffJoints;
		std::vector<std::string> jointNames;
		std::vector<std::string> touchLinks;
		std::map<std::string, std::string> controllers;
};

#endif /* ROS_PG_WS_2018_19_ARM_PLANNING_INCLUDE_ARM_PLANNING_ARMCONTROLPARAMETERLIST_H_ */
