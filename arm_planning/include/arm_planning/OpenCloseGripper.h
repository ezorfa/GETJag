/*
 * OpenGripper.h
 *
 *  Created on: Mar 27, 2019
 *      Author: Mohammed Afroze
 *
 *  This class implements the functionality to host the server to open/close gripper.
 *
 */

#ifndef ROS_PG_WS_2018_19_ARM_PLANNING_INCLUDE_ARM_PLANNING_OPENCLOSEGRIPPER_H_
#define ROS_PG_WS_2018_19_ARM_PLANNING_INCLUDE_ARM_PLANNING_OPENCLOSEGRIPPER_H_

#include <memory>
#include <string>

#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>

#include <arm_planning/ActionServerInterface.h>
#include <arm_planning/Executor.h>
#include <handling_msgs/GoapMsgAction.h>

class OpenCloseGripperAction : public ActionServerInterface
{
	public:
		OpenCloseGripperAction (ros::NodeHandle& nh, std::shared_ptr<ArmControlParameterList>& paramList, std::shared_ptr<Executor>& exe);
		virtual ~OpenCloseGripperAction ();
		void goalCB ();
		void setSucceeded () override;
		void setFeedback (double f) override;
		void setPreempted () override;
		bool isActive () override;

	private:
		std::string convertGoalToGoapMsg (const geometry_msgs::Pose& pose) const;
		void preemptCB ();
		using GripperServer = actionlib::SimpleActionServer<handling_msgs::GoapMsgAction>;
		JoystickMappings joyMappings;
		ros::Publisher pub;
		ros::NodeHandle nh;
		Executor::Ptr exe;
		ArmControlParameterList::Ptr paramList;
		Parameters staticParams;
		std::unique_ptr<GripperServer> server;
};

#endif /* ROS_PG_WS_2018_19_ARM_PLANNING_INCLUDE_ARM_PLANNING_OPENCLOSEGRIPPER_H_ */
