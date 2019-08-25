/*
 * DropAction.h
 *
 *  Created on: Mar 27, 2019
 *      Author: mohammed
 *
 *  This class sets up drop object action server. This action is called after object has been grabbed.
 *  The main functionality is to open the gripper and remove all the objects from the current planning scene.
 *  Also detaches the object from the tool.
 */

#ifndef ROS_PG_WS_2018_19_ARM_PLANNING_INCLUDE_ARM_PLANNING_DROPACTION_H_
#define ROS_PG_WS_2018_19_ARM_PLANNING_INCLUDE_ARM_PLANNING_DROPACTION_H_

#include <memory>
#include <string>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>

#include <arm_planning/ActionServerInterface.h>
#include <arm_planning/Executor.h>
#include <arm_planning/Planning.h>
#include <handling_msgs/GoapMsgAction.h>
#include <handling_msgs/ToggleGUIExecuteCancelBtn.h>

class DropObjectAction : public ActionServerInterface
{
	public:
		DropObjectAction (ros::NodeHandle& nh, std::shared_ptr<ArmControlParameterList>& paramList, Planning::Ptr& plan, std::shared_ptr<Executor>& exe);
		virtual ~DropObjectAction ();
		void goalCB ();
		void setSucceeded () override;
		void setFeedback (double f) override;
		void setPreempted () override;
		bool isActive () override;

	private:
		std::string convertGoalToGoapMsg (const geometry_msgs::Pose& pose) const;
		void preemptCB ();
		using DropServer = actionlib::SimpleActionServer<handling_msgs::GoapMsgAction>;

		ros::NodeHandle nh;
		ros::ServiceClient client;
		Executor::Ptr exe;
		Planning::Ptr plan;
		ArmControlParameterList::Ptr paramList;
		Parameters staticParams;
		std::unique_ptr<actionlib::SimpleActionClient<handling_msgs::GoapMsgAction>> gripperGoalClient;
		std::unique_ptr<DropServer> server;
};

#endif /* ROS_PG_WS_2018_19_ARM_PLANNING_INCLUDE_ARM_PLANNING_DROPACTION_H_ */
