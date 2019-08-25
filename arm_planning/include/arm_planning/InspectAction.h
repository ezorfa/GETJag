/*
 * InspectAction.h
 *
 *  Created on: Feb 7, 2019
 *      Author: mohammed
 */

#ifndef ROS_PG_WS_2018_19_ARM_PLANNING_INCLUDE_ARM_PLANNING_INSPECTACTION_H_
#define ROS_PG_WS_2018_19_ARM_PLANNING_INCLUDE_ARM_PLANNING_INSPECTACTION_H_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Pose.h>
#include <ros/callback_queue.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/spinner.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>

#include <arm_planning/ActionServerInterface.h>
#include <arm_planning/Planning.h>
#include <arm_planning/Executor.h>
#include <handling_msgs/PoseGoalAction.h>
#include <handling_msgs/GoapMsgAction.h>
#include <handling_msgs/ToggleGUIExecuteCancelBtn.h>

class InspectAction : public ActionServerInterface
{
	public:
		InspectAction (ros::NodeHandle& nh, ArmControlParameterList::Ptr& paramList, Executor::Ptr& exe, Planning::Ptr& plan);
		virtual ~InspectAction ();
		void goalCB ();
		void goalPreemptCB ();
		void setSucceeded () override;
		void doTransform(const geometry_msgs::Pose in, geometry_msgs::Pose &out,const std::string target="/GETjag/base_link",const std::string base="/GETjag/odom");
		void setFeedback (double f) override;
		void setPreempted () override;
		bool isActive () override;

	private:
		void cleanUp ();
		bool isCancelRequested ();
		void publishObjectGrabbed (bool msg);
		std::string convertGoalToGoapMsg (const geometry_msgs::Pose& pose) const;
		using InspectServer = actionlib::SimpleActionServer<handling_msgs::GoapMsgAction>;
		ros::NodeHandle nh;
		ros::Publisher objectGrabbedPub;
		ros::ServiceClient client;
		ros::NodeHandle nhPrivate;
		ros::CallbackQueue callbackQueue;
		ros::AsyncSpinner spinner;
		std::shared_ptr<tf::TransformListener> listener;
		Executor::Ptr exe;
		Planning::Ptr plan;
		ArmControlParameterList::Ptr paramList;
		Parameters staticParams;
		std::unique_ptr<InspectServer> server;
		std::unique_ptr<actionlib::SimpleActionClient<handling_msgs::GoapMsgAction>> poseGoalClient;
		std::unique_ptr<actionlib::SimpleActionClient<handling_msgs::GoapMsgAction>> gripperGoalClient;
		std::map<std::string, std::vector<double>> inspects;
		std::vector<std::string> objectID;
};

#endif /* ROS_PG_WS_2018_19_ARM_PLANNING_INCLUDE_ARM_PLANNING_INSPECTACTION_H_ */
