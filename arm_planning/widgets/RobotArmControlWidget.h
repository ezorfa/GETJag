/*
 * RobotArmControlWidget.h
 *
 *  Created on: Mar 21, 2019
 *      Author: mohammed
 */

#ifndef ROS_GETBOT_SRC_OPERATOR_INTERFACE_WIDGETS_ROBOTARMCONTROLWIDGET_H_
#define ROS_GETBOT_SRC_OPERATOR_INTERFACE_WIDGETS_ROBOTARMCONTROLWIDGET_H_


#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_srvs/Empty.h>
#endif


#include <QtCore/QtPlugin>
#include <QtWidgets/QWidget>

#include <core/WidgetPluginInterface.h>
#include <core/BaseWidget.h>
#include <ui_RobotArmControlWidget.h>

#include <actionlib/client/simple_action_client.h>
#include <handling_msgs/GoapMsgAction.h>
#include <handling_msgs/GetCurrentPose.h>
#include <yaml-cpp/yaml.h>
#include <ros/package.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <handling_msgs/JoyControlArmWristReset.h>
#include <handling_msgs/ObjectGrabbed.h>
#include <handling_msgs/ToggleGUIExecuteCancelBtn.h>


class RobotArmControlWidget : public BaseWidget, private Ui_RobotArmControlWidget
{

		Q_OBJECT
		Q_INTERFACES(WidgetPluginInterface)
		Q_PLUGIN_METADATA(IID "RobotArmControlWidget")

		public:

		RobotArmControlWidget ();
		virtual ~RobotArmControlWidget ();

		QWidget* createInstance () override;
		void init () override;
		void refresh () override;
		void reset () override;

		std::string currentAction;
		std::string defaultListOption ;
		std::shared_ptr<actionlib::SimpleActionClient<handling_msgs::GoapMsgAction>> currentActionClient = nullptr;


	private:
		std::shared_ptr<actionlib::SimpleActionClient<handling_msgs::GoapMsgAction>> goHomeClient;
		std::shared_ptr<actionlib::SimpleActionClient<handling_msgs::GoapMsgAction>> graspGoalClient;
		std::shared_ptr<actionlib::SimpleActionClient<handling_msgs::GoapMsgAction>> inspectGoalClient;
		std::shared_ptr<actionlib::SimpleActionClient<handling_msgs::GoapMsgAction>> extractGoalClient;
		std::shared_ptr<actionlib::SimpleActionClient<handling_msgs::GoapMsgAction>> poseGoalClient;
		std::shared_ptr<actionlib::SimpleActionClient<handling_msgs::GoapMsgAction>> gripperGoalClient;
		std::shared_ptr<actionlib::SimpleActionClient<handling_msgs::GoapMsgAction>> dropGoalClient;

		QTimer *timer;
		bool isPoseGoalExecuted = false;

	private Q_SLOTS:

		void slotBtnResetClicked ();
		void slotBtnExecuteClicked ();
		void slotBtnCancelClicked ();
		void slotBtnClearOctomapClicked();
		void slotTaskListWidgetClicked ();
		void removeAllListWidgetItems ();
		void objectGrabbedGraspCB(const handling_msgs::ObjectGrabbedConstPtr& msg);
		void objectGrabbedExtractCB(const handling_msgs::ObjectGrabbedConstPtr& msg);
		bool toggleExeCancelBtnCB(handling_msgs::ToggleGUIExecuteCancelBtn::Request  &req, handling_msgs::ToggleGUIExecuteCancelBtn::Response &res);
		void slotChangeControlMode ();
		void addComboBoxItems (const std::string filename);
		std::string convertGoalToGoapMsg (std::string task, const std::string objectID);
		std::string convertGoalToPoseGoal (tf2::Quaternion orientation);
		std::tuple<std::vector<std::string>, std::vector<std::string>> loadParamsFromFile (const std::string yamlFileName);
		void slotChangeToAutoMode ();
		void slotChangeToJoyMode ();
		void slotChangeToRobotRefFrame ();
		void slotChangeToCameraRefFrame ();
		void slotBtnGripperOpenClicked();
		void slotBtnGripperCloseClicked();
		void slotBtnCurrentPoseClicked();
		void slotBtnWristResetClicked();
		void goHomeCallBack (const handling_msgs::GoapMsgActionFeedbackConstPtr& feedback);
		void extractCallBack (const handling_msgs::GoapMsgActionFeedbackConstPtr& feedback);
		void inspectCallBack(const handling_msgs::GoapMsgActionFeedbackConstPtr& feedback);
		void grabCallBack (const handling_msgs::GoapMsgActionFeedbackConstPtr& feedback);
		void dropCallBack (const handling_msgs::GoapMsgActionFeedbackConstPtr& feedback);
		void poseGoalCallBack (const handling_msgs::GoapMsgActionFeedbackConstPtr& feedback);
		void progressBarCB(double fb);
		void update ();

};

#endif /* ROS_GETBOT_SRC_OPERATOR_INTERFACE_WIDGETS_ROBOTARMCONTROLWIDGET_H_ */
