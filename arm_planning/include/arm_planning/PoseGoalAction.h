/*
 * PoseGoalAction.h
 *
 *  Created on: Feb 7, 2019
 *      Author: Mohammed Afroze
 *
 *  This class implements functionality of moving the arm to a specified pose.
 *  Hosts an ROS action server to recieve a goal point.
 */

#ifndef ROS_PG_WS_2018_19_ARM_PLANNING_INCLUDE_ARM_PLANNING_POSEGOALACTION_H_
#define ROS_PG_WS_2018_19_ARM_PLANNING_INCLUDE_ARM_PLANNING_POSEGOALACTION_H_

#include <memory>

#include <actionlib/server/simple_action_server.h>
#include <ros/ros.h>

#include <arm_planning/ActionServerInterface.h>
#include <arm_planning/Planning.h>
#include <arm_planning/Executor.h>
#include <handling_msgs/PoseGoalAction.h>
#include <handling_msgs/GoapMsgAction.h>
#include <handling_msgs/ToggleGUIExecuteCancelBtn.h>
#include <moveit/move_group_interface/move_group_interface.h>

class PoseGoalAction : public ActionServerInterface
{
	public:
		PoseGoalAction (ros::NodeHandle&, std::shared_ptr<ArmControlParameterList>& paramList, std::shared_ptr<Executor>&, std::shared_ptr<Planning>&);
		virtual ~PoseGoalAction ();
		void poseGoalCB ();
		void poseGoalPreemptCB ();
		void setFeedback (double f) override;
		void setSucceeded () override;
		void setPreempted () override;
		bool isActive () override;

	private:
		using PoseGoalServer = actionlib::SimpleActionServer<handling_msgs::GoapMsgAction>;

		Parameters staticParams;
		Executor::Ptr exe;
		Planning::Ptr plan;
		ros::NodeHandle nh;
		ros::ServiceClient client;
		ArmControlParameterList::Ptr paramList;
		std::unique_ptr<PoseGoalServer> server;
		std::unique_ptr<moveit::planning_interface::MoveGroupInterface> moveGroup;

		ros::CallbackQueue callbackQueue;
		ros::NodeHandle nodeHandleMoveGroup;
		ros::AsyncSpinner spinner;
		handling_msgs::GoapMsgActionFeedback feedback;

		std::string baseLinkRefFrame;

};

#endif /* ROS_PG_WS_2018_19_ARM_PLANNING_INCLUDE_ARM_PLANNING_POSEGOALACTION_H_ */
