/*
 * Executor.h
 *
 *  Created on: Jun 12, 2018
 *      Author: Mohammed Afroze
 *
 *  The Executor class has the impl for executing a trajectory received after planning with
 *  moveit. This trajectory recieved is used to execute on the position controllers.
 *  Also contains impl to open/close gripper.
 *
 */

#ifndef ARM_PLANNING_EXECUTE_H
#define ARM_PLANNING_EXECUTE_H

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <dynamixel_controllers/SetSpeed.h>
#include <dynamixel_msgs/JointState.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/service_client.h>
#include <ros/subscriber.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <arm_planning/ActionServerInterface.h>
#include <arm_planning/JointStateSubscriber.h>
#include <arm_planning/ArmControlParameterList.h>

class Executor
{
	public:
		Executor (ros::NodeHandle& nh, std::shared_ptr<ArmControlParameterList>& paramList, std::shared_ptr<JointStateSubscriber>& jointStates);
		~Executor () = default;

		void executePlan (moveit_msgs::RobotTrajectory trajectory, ActionServerInterface* server = nullptr);
		void monitorGrabStatus ();
		void abortExecution ();
		void execute (const std::string joint, const double value);
		void execute (std::map<std::string, double>& jointAngles);
		void setSpeed (const std::string joint, const double speed);
		void setupGripperControl (ArmControlParameterList::GripperMode mode, double pos, ActionServerInterface* gripperAction = nullptr);
		bool isBusy ();
		bool checkIfGoalPosReached ();
		bool isObjectGrabbed ();
		void setServer (ActionServerInterface* actionServer = nullptr);

		using Ptr = std::shared_ptr<Executor>;

	private:
		void execute (const sensor_msgs::JointStateConstPtr&);
		void calculateSpeed (std::map<std::string, double>& jointAngles);
		bool isJointOverloaded (std::string l_jointName, const double value);
		void gripperTimerCallback (const ros::TimerEvent& event);

		ActionServerInterface* server = nullptr;
		ActionServerInterface* gripperServer = nullptr;
		ActionServerInterface* actionServer = nullptr;
		JointStateSubscriber::Ptr jointState;
		ArmControlParameterList::Ptr paramList;
		ArmControlParameterList::GripperMode gripperMode;
		Parameters staticParams;

		std::map<std::string, ros::Publisher> controllerPublisher;
		std::map<std::string, std::string> controllerMap;
		std::map<std::string, ros::ServiceClient> setSpeedService;
		std::map<std::string, double> currentJointLoad, currentState, goalWayPoint, maxJointSpeedLimits;
		std::map<std::string, double> calculatedSpeed, targetJointAngles;
		std::vector<std::string> jointNames, controllers, joints;
		std::string speedTopic, gripper, gripper_controller;

		ros::Timer grabingTimer;
		ros::Subscriber jointStateSub;
		dynamixel_controllers::SetSpeed speedData;
		trajectory_msgs::JointTrajectory traj;

		const double MIN_SPEED = 0.05;
		int percentageTrajExecuted = 0;
		int currentWayPoint = 0;
		int counterGrabingTimer = 0;
		int loopCount = 0;
		double gripperGoalPos = 0;
		bool objectGrabbed = false;
		bool objectDropped = false;
		bool hasActiveGoal = false;
		bool isNearGoalPos = true;

};

#endif
