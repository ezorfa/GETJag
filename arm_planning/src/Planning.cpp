/*
 * Planning.cpp
 *
 *  Created on: Mar 22, 2019
 *      Author: Mohammed Afroze
 */
#include <eigen3/Eigen/Eigen>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/frames_io.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/current_state_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <arm_planning/Planning.h>

Planning::Planning (ros::NodeHandle& nh, std::shared_ptr<ArmControlParameterList>& paramList, std::shared_ptr<JointStateSubscriber>& jointStates)
		: jointState (jointStates), spinner (1, &callbackQueue), paramList (paramList)
{
	using MoveItInterface = moveit::planning_interface::MoveGroupInterface;
	planningSceneInterface = std::make_shared<moveit::planning_interface::PlanningSceneInterface> ();
	jointNames = paramList->getAllJoints ();
	touchLinks = paramList->getTouchLinks ();
	staticParams = paramList->getStaticParams ();
	baseLinkRefFrame = staticParams.baseLinkRefFrame;
	nodeHandleMoveGroup.setCallbackQueue (&callbackQueue);
	currentPoseService = nh.advertiseService ("dexterity/get_current_pose_endeffector", &Planning::queryCurrentPose, this);
	moveGroup = std::make_unique<MoveItInterface> (MoveItInterface::Options ("GETjag_arm", "robot_description", nodeHandleMoveGroup));
	moveGroup->startStateMonitor ();
	spinner.start ();
}

// Query current pose of the end effector through the service.
bool Planning::queryCurrentPose (handling_msgs::GetCurrentPose::Request &req, handling_msgs::GetCurrentPose::Response &res)
{
	geometry_msgs::Pose pose = moveGroup->getCurrentPose ().pose;
	auto rpyVector = moveGroup->getCurrentRPY ();
	res.pose = pose;
	res.rpy = rpyVector;
	return true;
}

void Planning::setGrabbedObject (const std::string object) noexcept
{
	grabbedObject = object;
}

std::string Planning::getGrabbedObject ()
{
	return grabbedObject;
}
void Planning::addMeshCollisionObjects (const std::vector<std::string>& objectID, const std::string& meshPath, geometry_msgs::Pose pose)
{
	moveit_msgs::CollisionObject object;
	object.header.frame_id = "GETjag/base_link";
	object.id = objectID.front ();
	shapes::Mesh* mesh;
	// The scaling is required for the box, as the mesh found in our models of meshes for the box is too large. Its a workaround.
	const Eigen::Vector3d scale (0.15, 0.15, 0.15); // TODO: Should be removed as soon as we have right dimension box available.
	if (objectID.front () == "box")
		mesh = shapes::createMeshFromResource (meshPath, scale);
	else
		mesh = shapes::createMeshFromResource (meshPath);

	shape_msgs::Mesh objectMesh;
	shapes::ShapeMsg objectMeshMsg;
	shapes::constructMsgFromShape (mesh, objectMeshMsg);
	objectMesh = boost::get<shape_msgs::Mesh> (objectMeshMsg);
	object.meshes.resize (1);
	object.meshes[0] = objectMesh;
	object.mesh_poses.resize (1);
	object.mesh_poses[0].position.x = pose.position.x;
	object.mesh_poses[0].position.y = pose.position.y;
	object.mesh_poses[0].position.z = pose.position.z;
	object.mesh_poses[0].orientation.x = pose.orientation.x;
	object.mesh_poses[0].orientation.y = pose.orientation.y;
	object.mesh_poses[0].orientation.z = pose.orientation.z;
	object.mesh_poses[0].orientation.w = pose.orientation.w;
	object.meshes.push_back (objectMesh);
	object.mesh_poses.push_back (object.mesh_poses[0]);
	object.operation = object.ADD;
	collisionObjects.push_back (object);
	planningSceneInterface->applyCollisionObjects (collisionObjects);
}

void Planning::removeCollisionObjects (const std::vector<std::string>& objectID)
{
//	planningSceneInterface->removeCollisionObjects (objectID);
	moveit_msgs::CollisionObject object;
	object.header.frame_id = "GETjag/base_link";
	object.id = objectID.front ();
	object.operation = object.REMOVE;
	collisionObjects.push_back (object);
	planningSceneInterface->applyCollisionObjects (collisionObjects);
	ros::Duration (2).sleep ();
}

void Planning::attachObjectToTool (const std::string& objectID)
{
	if (touchLinks.empty ())
	{
		ROS_ERROR("Touch links are empty");
	}
	moveGroup->attachObject (objectID, "GETjag/arm_wrist_roll_link", touchLinks);
}

void Planning::dettachObjectFromTool (const std::string& objectID)
{
	moveGroup->detachObject (objectID);
}

bool Planning::planToHome (moveit_msgs::RobotTrajectory& traj)
{
	moveGroup->setStartState (*moveGroup->getCurrentState ());
	moveGroup->clearPoseTargets ();
	moveGroup->setNumPlanningAttempts (staticParams.moveitMaxPlanningAttempts);
	moveGroup->setMaxVelocityScalingFactor (staticParams.moveitMaxVelocityScalingFactor);
	moveGroup->setGoalTolerance (staticParams.moveitGoalTolerance);
	moveGroup->setPoseReferenceFrame (baseLinkRefFrame);
	const std::map<std::string, double> homeJointsMap = moveGroup->getNamedTargetValues ("GETjag_pose_home");
	moveGroup->setJointValueTarget (homeJointsMap);
	moveit::planning_interface::MoveGroupInterface::Plan plannedPath;
	bool GoHomesuccess = false;
	try
	{
		GoHomesuccess = (moveGroup->plan (plannedPath) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	}
	catch (ros::Exception& e)
	{
		std::cout << "Exception : " << e.what () << std::endl;
	}
	ROS_INFO_NAMED("Plan", "Arm Planning to go to home position : %s", GoHomesuccess ? "SUCCESS" : "FAILED");
	if (GoHomesuccess and plannedPath.trajectory_.joint_trajectory.points.size () > 2)
	{
		moveit_msgs::RobotTrajectory tempTraj;
		int lastPoint = plannedPath.trajectory_.joint_trajectory.points.size () - 1;
		traj.joint_trajectory = plannedPath.trajectory_.joint_trajectory;
		// As the trajectory generated by moveit, by default has zero velocity for the first point, so, its better to
		// set the velocities of the next point to the first point.
		traj.joint_trajectory.points[0].velocities = traj.joint_trajectory.points[1].velocities;
		// similar case applies as the before comment.
		traj.joint_trajectory.points[lastPoint].velocities = traj.joint_trajectory.points[lastPoint - 1].velocities;
		return true;
	}
	return false;
}

/* This function is use to plan to a specific pose supplied as a target pose and a container to store the trajectory
 * generated.
 *
 * plans by two methods. one is using regular plan function provided by moveGroup->plan().
 * And the other is by using computeCartesianPath().
 *
 *
 * */
bool Planning::plan (geometry_msgs::Pose targetPose, moveit_msgs::RobotTrajectory& traj, bool collisions)
{
	moveGroup->setStartState (*moveGroup->getCurrentState ());
	moveGroup->clearPoseTargets ();
	moveGroup->setNumPlanningAttempts (staticParams.moveitMaxPlanningAttempts);
	moveGroup->setMaxVelocityScalingFactor (staticParams.moveitMaxVelocityScalingFactor);
	moveGroup->setGoalTolerance (staticParams.moveitGoalTolerance);
	moveGroup->setPoseReferenceFrame (baseLinkRefFrame);
	moveGroup->setPoseTarget (targetPose);
	moveit::planning_interface::MoveGroupInterface::Plan plannedPath;
	bool success = false;
	if (collisions)
	{
		success = (moveGroup->plan (plannedPath) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
		ROS_INFO_NAMED("Plan", "Arm Planning to goal: %s", success ? "SUCCESS" : "FAILED");
		traj.joint_trajectory = plannedPath.trajectory_.joint_trajectory;
	}
	// This is a workaround solution for disabling collision avoidance. But this is not the standard of doing the same.
	// By default, moveit planner automatically takes into account the collisions in the planning scene.
	else
	{
		ROS_INFO_STREAM("Planning by computeCartesianPath");
		std::vector<geometry_msgs::Pose> vector;
		vector.push_back (targetPose);
		double fraction = 0;
		for (int attempts = 0; attempts < 5; attempts++)
		{
			fraction = moveGroup->computeCartesianPath (vector, staticParams.moveitEEFStep, 0.0, traj, false);
			if (fraction > 0.9)
			{
				success = true;
				break;
			}
		}
	}
	if (success and traj.joint_trajectory.points.size () > 2)
	{
		// set the first and last point's velocities to the next/previous velocities to avoid traj execution startup problems.
		int lastPoint = traj.joint_trajectory.points.size () - 1;
		traj.joint_trajectory.points[0].velocities = traj.joint_trajectory.points[1].velocities;
		traj.joint_trajectory.points[lastPoint].velocities = traj.joint_trajectory.points[lastPoint - 1].velocities;
		return true;
	}
	return false;
}

void Planning::setCollisionAvoidance (bool collisions)
{
	collisionAvoidance = collisions;
}

/* This function is used for planning with joystick. A target pose initially is the starting point of the end-effector. As try to
 * navigate the arm using joystick the relevant increments is added to the target pose.
 *
 * Replanned for every small increments.
 *
 * target pose is resetted to the current pose of the end effector whenever resetTarget is true.
 *
 *
 * */
bool Planning::plan (std::map<std::string, double>& increments, const std::string refFrame, std::map<std::string, double>& targetJointAngles, bool resetTarget)
{
	moveGroup->setPlanningTime (0.5);
	if (resetTarget or inConstraintsRegion)
	{
		ROS_INFO_STREAM("Resetting Target Pose");
		targetPose = moveGroup->getCurrentPose ().pose;
		currentRPY = moveGroup->getCurrentRPY ();
		inConstraintsRegion = false;
	}
	if (planningAttempts > 0)
	{
		incrementTarget = false;
	}
	// targetpose is resetted after every 5 failed attempts to plan.
	if (planningAttempts > 5 )
	{
		ROS_INFO_STREAM("Resetting Target Pose after 5 unsuccessful planningAttempts");
		targetPose = moveGroup->getCurrentPose ().pose;
		currentRPY = moveGroup->getCurrentRPY ();
	}
	moveGroup->setStartStateToCurrentState ();
	moveGroup->setGoalTolerance (staticParams.moveitGoalTolerance);
	moveGroup->setPoseReferenceFrame (baseLinkRefFrame);
	if (incrementTarget)
	{
		// Setting target pose, which in turn is used by the planner to plan with respect to End-Effector frame.
		if (refFrame == staticParams.endEffectorRefFrame)
		{
			KDL::Rotation t_temp_rot_difFrame;
			KDL::Frame l_temp_actFramme;
			l_temp_actFramme = KDL::Frame (KDL::Vector (targetPose.position.x, targetPose.position.y, targetPose.position.z));
			t_temp_rot_difFrame = KDL::Rotation::RPY (currentRPY[0], currentRPY[1], currentRPY[2]);
			l_temp_actFramme.M = t_temp_rot_difFrame;

			KDL::Frame l_temp_difFrame;
			l_temp_difFrame = KDL::Frame (KDL::Vector (increments["x"], increments["y"], increments["z"]));

			KDL::Frame l_temp_targetFrame;
			l_temp_targetFrame = l_temp_actFramme * l_temp_difFrame;

			targetPose.position.x = l_temp_targetFrame.p.x ();
			targetPose.position.y = l_temp_targetFrame.p.y ();
			targetPose.position.z = l_temp_targetFrame.p.z ();

			double temp_rot_x, temp_rot_y, temp_rot_z, temp_rot_w;
			l_temp_targetFrame.M.GetQuaternion (temp_rot_x, temp_rot_y, temp_rot_z, temp_rot_w);
			targetPose.orientation.x = temp_rot_x;
			targetPose.orientation.y = temp_rot_y;
			targetPose.orientation.z = temp_rot_z;
			targetPose.orientation.w = temp_rot_w;
		}
		// Setting target pose, which is used by the planner to plan with respect to baseLinkFrame.
		else
		{
			targetPose.position.x += increments["x"];
			targetPose.position.y += increments["y"];
			targetPose.position.z += increments["z"];
		}
	}

	// moveGroup->setPoseTarget(targetPose);
	// moveit::planning_interface::MoveGroupInterface::Plan plannedPath;

	// bool success = (moveGroup->plan(plannedPath)== moveit::planning_interface::MoveItErrorCode::SUCCESS);

	// if (plannedPath.trajectory_.joint_trajectory.points.size() > 1)
	// {
	// planningAttempts = 0;
	// success = true;
	// int i= 0;
	// for (auto& joint : jointNames)
	// {
	// targetJointAngles[joint] = plannedPath.trajectory_.joint_trajectory.points.back().positions[i];
	// i++;
	// }

	// }

	moveit_msgs::RobotTrajectory traj;
	std::vector<geometry_msgs::Pose> vector;
	vector.push_back (targetPose);
	bool success = false;

	double fraction = moveGroup->computeCartesianPath (vector, staticParams.moveitEEFStep, 0.0, traj, collisionAvoidance);

	if (traj.joint_trajectory.points.size () > 1)
	{
		success = true;
		int i = 0;
		for (auto& joint : jointNames)
		{
			if (joint != staticParams.gripper)
			{
				// Only the last point in the trajectory is taken as the distance between points are very small to be considered.
				targetJointAngles[joint] = traj.joint_trajectory.points.back ().positions[i];
			}
			i++;
		}
		planningAttempts = 0;
		incrementTarget = true;
	}
	else
	{
		planningAttempts++;
	}

	// Constraints, that enables the arm to move through the troublesome region smoothly while using joystick.
	if (refFrame == staticParams.baseLinkRefFrame)
	{
		if ((increments["x"] != 0 and increments["y"] == 0 and increments["z"] == 0)
		//	or (increments["x"] == 0 and increments["y"] != 0 and increments["z"] == 0)
		//	or (increments["x"] == 0 and increments["y"] == 0 and increments["z"] != 0)
		)
		{
			if (targetPose.position.x > staticParams.joyPlanningXLowerLimit and targetPose.position.x < staticParams.joyPlanningXHigherLImit)
			{
				inConstraintsRegion = true;
				targetJointAngles["GETjag/arm_wrist_yaw_joint"] = 0;
				targetJointAngles["GETjag/arm_wrist_roll_joint"] = 0;
				targetJointAngles["GETjag/arm_base_joint"] = 0;
			}
		}
	}
	// with respect to the camera frame.
	else
	{
		if ((increments["x"] != 0 and increments["y"] == 0 and increments["z"] == 0)
		//		or (increments["x"] == 0 and increments["y"] != 0 and increments["z"] == 0)
		//              or (increments["x"] == 0 and increments["y"] == 0 and increments["z"] != 0)
		)
		{
			if (targetPose.position.x > staticParams.joyPlanningXLowerLimit and targetPose.position.x < staticParams.joyPlanningXHigherLImit)
			{
				inConstraintsRegion = true;
				targetJointAngles["GETjag/arm_wrist_yaw_joint"] = 0;
				targetJointAngles["GETjag/arm_wrist_roll_joint"] = 0;
				targetJointAngles["GETjag/arm_base_joint"] = 0;
			}
		}
	}

	return success;
}

