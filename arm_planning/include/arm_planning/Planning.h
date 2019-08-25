/*
 * Move.h
 *
 *  Created on: Jun 12, 2018
 *      Author: Mohammed Afroze
 *
 *  This header file contains the class which implements the functionality to plan a trajectory using
 *  Moveit! motion planning framework.
 *
 *  It also has member functions to attach/dettach collision objects from the planning scene.
 */

#ifndef ARM_PLANNING_PLANNING_H
#define ARM_PLANNING_PLANNING_H

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <ros/ros.h>
#include <dynamixel_controllers/SetSpeed.h>

#include <arm_planning/JointStateSubscriber.h>
#include <arm_planning/ArmControlParameterList.h>
#include <arm_planning/ActionServerInterface.h>
#include <handling_msgs/GetCurrentPose.h>

class Planning
{
	public:
		Planning (ros::NodeHandle& nh, std::shared_ptr<ArmControlParameterList>& paramList, std::shared_ptr<JointStateSubscriber>& jointStates);
		~Planning () = default;
		bool plan (geometry_msgs::Pose targetPose, moveit_msgs::RobotTrajectory& traj, bool collisions);
		bool plan (std::map<std::string, double>& increments, const std::string refFrame, std::map<std::string, double>& targetJointAngles, bool clearTargetPose);
		bool planToHome (moveit_msgs::RobotTrajectory& traj);
		void setCollisionAvoidance (bool collisions);
		void attachObjectToTool (const std::string& objectID);
		void dettachObjectFromTool (const std::string& objectID);
		void addMeshCollisionObjects (const std::vector<std::string>& objectID, const std::string& meshPath, geometry_msgs::Pose pose);
		void removeCollisionObjects (const std::vector<std::string>& objectID);
		bool queryCurrentPose (handling_msgs::GetCurrentPose::Request &req, handling_msgs::GetCurrentPose::Response &res);
		void setGrabbedObject (const std::string object) noexcept;
		std::string getGrabbedObject ();
		using Ptr = std::shared_ptr<Planning>;

	private:
		Parameters staticParams;
		ArmControlParameterList::Ptr paramList;
		JointStateSubscriber::Ptr jointState;
		std::unique_ptr<moveit::planning_interface::MoveGroupInterface> moveGroup;
		std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planningSceneInterface;
		ActionServerInterface* server = nullptr;
		// Separate node handle for movegroup and callbackqueue service.
		ros::CallbackQueue callbackQueue;
		ros::NodeHandle nodeHandleMoveGroup;
		ros::AsyncSpinner spinner;

		ros::ServiceClient octomapService;
		ros::ServiceServer currentPoseService;

		std::vector<moveit_msgs::CollisionObject> collisionObjects;
		std::vector<std::string> jointNames;
		std::vector<std::string> touchLinks;
		std::vector<double> currentRPY;
		geometry_msgs::Pose targetPose;
		std::string goHomeActionName, goToPoseActionName, planningGroup, baseLinkRefFrame, endEffectorRefFrame, grabbedObject;
		int planningAttempts = 0;
		bool collisionAvoidance = true;
		bool incrementTarget = true;
		bool inConstraintsRegion = false;
};

#endif /* ARM_PLANNING_PLANNING_H */
