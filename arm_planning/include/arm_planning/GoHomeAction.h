/*
 * ActionWrapper.hpp
 *
 *  Created on: Feb 5, 2019
 *      Author: mohammed
 */

#ifndef ARM_PLANNING_INCLUDE_GOHOMEACTION_H_
#define ARM_PLANNING_INCLUDE_GOHOMEACTION_H_

#include <memory>
#include <string>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <ros/node_handle.h>

#include <arm_planning/ActionServerInterface.h>
#include <arm_planning/Executor.h>
#include <arm_planning/Planning.h>
#include <handling_msgs/GoapMsgAction.h>

class GoHomeAction : public ActionServerInterface
{
	public:
		GoHomeAction (ros::NodeHandle&, std::shared_ptr<ArmControlParameterList>& paramList, std::shared_ptr<Executor>&, std::shared_ptr<Planning>&);
		virtual ~GoHomeAction ();
		void goHomeCB ();
		void goHomePreemptCB ();
		void setFeedback (double f) override;
		void setSucceeded () override;
		void setPreempted () override;
		bool isActive () override;

	private:
		bool isCancelRequested ();
		using GoHomeServer = actionlib::SimpleActionServer<handling_msgs::GoapMsgAction>;
		Parameters staticParams;
		Executor::Ptr exe;
		Planning::Ptr plan;
		ros::NodeHandle nh;
		ArmControlParameterList::Ptr paramList;
		std::unique_ptr<GoHomeServer> server;
		std::unique_ptr<actionlib::SimpleActionClient<handling_msgs::GoapMsgAction>> gripperGoalClient;
		handling_msgs::GoapMsgActionFeedback feedback;
		std::string baseLinkRefFrame;
};

#endif /* ARM_PLANNING_INCLUDE_GOHOMEACTION_H_ */
