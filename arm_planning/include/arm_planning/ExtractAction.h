/*
 * ExtractAction.h
 *
 *  Created on: Mar 27, 2019
 *      Author: Mohammed Afroze
 *
 *  This class implements a pipeline/ sequence of actions to be performed in order to do Extraction.
 *  Hosts ROS action server to which goal can be sent using an action client.
 */

#ifndef ROS_PG_WS_2018_19_ARM_PLANNING_INCLUDE_ARM_PLANNING_EXTRACTACTION_H_
#define ROS_PG_WS_2018_19_ARM_PLANNING_INCLUDE_ARM_PLANNING_EXTRACTACTION_H_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>

#include <arm_planning/ActionServerInterface.h>
#include <arm_planning/Executor.h>
#include <arm_planning/Planning.h>
#include <handling_msgs/PoseGoalAction.h>
#include <handling_msgs/GoapMsgAction.h>
#include <handling_msgs/ToggleGUIExecuteCancelBtn.h>

class ExtractAction : public ActionServerInterface
{
	public:
		ExtractAction (ros::NodeHandle& nh, ArmControlParameterList::Ptr& paramList, Executor::Ptr& exe, Planning::Ptr& plan);
		virtual ~ExtractAction ();
		void goalCB ();
		void goalPreemptCB ();
		void setSucceeded () override;
		void setPreempted () override;
		void setFeedback (double f) override;
		bool isActive () override;
		void doTransform (const geometry_msgs::Pose in, geometry_msgs::Pose &out, const std::string target = "/GETjag/base_link", const std::string base = "/GETjag/odom");

	private:
		void cleanUp ();
		bool isCancelRequested ();
		std::string convertGoalToGoapMsg (const geometry_msgs::Pose& pose) const;
		using ExtractServer = actionlib::SimpleActionServer<handling_msgs::GoapMsgAction>;

		ros::NodeHandle nh;
		ros::NodeHandle nhPrivate;
		ros::ServiceClient client;
		ros::Publisher objectGrabbedPub;
		ros::CallbackQueue callbackQueue;
		ros::AsyncSpinner spinner;
		Executor::Ptr exe;
		Planning::Ptr plan;
		ArmControlParameterList::Ptr paramList;
		Parameters staticParams;
		std::unique_ptr<ExtractServer> server;
		std::unique_ptr<actionlib::SimpleActionClient<handling_msgs::GoapMsgAction>> poseGoalClient;
		std::unique_ptr<actionlib::SimpleActionClient<handling_msgs::GoapMsgAction>> gripperGoalClient;

		std::shared_ptr<tf::TransformListener> listener;
		std::map<std::string, std::vector<double>> extractPoints;
		std::vector<std::string> objectID;
		bool isObjectAttacedToTool = false;

};

#endif /* ROS_PG_WS_2018_19_ARM_PLANNING_INCLUDE_ARM_PLANNING_EXTRACTACTION_H_ */
