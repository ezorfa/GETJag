/*
 * GrabAction.h
 *
 *  Created on: Feb 7, 2019
 *      Author: Mohammed Afroze
 *
 *  This class implements a pipeline/ sequence of actions to be performed in order to grab an object.
 *  Hosts ROS action server to which goal can be sent using an action client.
 */

#ifndef ROS_PG_WS_2018_19_ARM_PLANNING_INCLUDE_ARM_PLANNING_GRABACTION_H_
#define ROS_PG_WS_2018_19_ARM_PLANNING_INCLUDE_ARM_PLANNING_GRABACTION_H_

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

class GrabAction : public ActionServerInterface
{
	public:
		GrabAction (ros::NodeHandle& nh, ArmControlParameterList::Ptr& paramList, Executor::Ptr& exe, Planning::Ptr& plan);
		virtual ~GrabAction ();
		void goalCB ();
		void goalPreemptCB ();
		//Function to transform a pose from base frame to target frame.
		void doTransform (const geometry_msgs::Pose in, geometry_msgs::Pose &out, const std::string target = "/GETjag/base_link", const std::string base = "/GETjag/odom");
		void setSucceeded () override;
		void setFeedback (double f) override;
		void setPreempted () override;
		bool isActive () override;

	private:
		void cleanUp ();
		bool isCancelRequested ();
		void publishObjectGrabbed (bool msg);
		std::string convertGoalToGoapMsg (const geometry_msgs::Pose& pose) const;
		using GrabServer = actionlib::SimpleActionServer<handling_msgs::GoapMsgAction>;
		ros::NodeHandle nh;
		ros::ServiceClient client;
		ros::Publisher objectGrabbedPub;
		ros::NodeHandle nhPrivate;
		ros::CallbackQueue callbackQueue;
		ros::AsyncSpinner spinner;
		std::shared_ptr<tf::TransformListener> listener;
		Executor::Ptr exe;
		Planning::Ptr plan;
		ArmControlParameterList::Ptr paramList;
		Parameters staticParams;
		std::unique_ptr<GrabServer> server;
		std::unique_ptr<actionlib::SimpleActionClient<handling_msgs::GoapMsgAction>> poseGoalClient;
		std::unique_ptr<actionlib::SimpleActionClient<handling_msgs::GoapMsgAction>> gripperGoalClient;
		std::map<std::string, std::vector<double>> grasps;
		std::vector<std::string> objectID;
		bool isObjectAttacedToTool = false;
};

#endif /* ROS_PG_WS_2018_19_ARM_PLANNING_INCLUDE_ARM_PLANNING_GRABACTION_H_ */
