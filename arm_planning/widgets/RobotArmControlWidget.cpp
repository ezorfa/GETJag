#include <cstdlib>

#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/Bool.h>

#include <widgets/RobotArmControlWidget.h>
#include "actionlib/client/simple_goal_state.h"
#include "actionlib/client/simple_client_goal_state.h"

const std::string goHome_sub = "goHome_sub";
const std::string grab_sub = "grab_sub";
const std::string extract_sub = "extract_sub";
const std::string inspect_sub = "inspect_sub";
const std::string poseGoal_sub = "poseGoal_sub";
const std::string grab_restltSub = "grab_restltSub";
const std::string object_grabbed_grasp = "object_grabbed_grasp";
const std::string object_grabbed_extract = "object_grabbed_extract";
const std::string toggleSrv = "toggleSrv";
const std::string currentPoseService = "currentPoseService";
const std::string clearOctomapService = "clearOctomapService";
const std::string wristReset_sub = "wristReset_sub";
const std::string Reset_sub = "Reset_sub";
const std::string drop_sub = "drop_sub";
const std::string automatic = "Automatic";
const std::string joystick = "Joystick";

RobotArmControlWidget::RobotArmControlWidget ()
{

	setupUi (this);
	joycontrol->setProperty ("checked", true);
	manualMode->setProperty ("checked", true);
	robotFrame->setProperty ("checked", true);

	Q_EMIT progressBar->setValue (0);

	goHomeClient = std::make_shared<actionlib::SimpleActionClient<handling_msgs::GoapMsgAction>> ("/dexterity/goHome", true);
	poseGoalClient = std::make_unique<actionlib::SimpleActionClient<handling_msgs::GoapMsgAction>> ("/dexterity/goalPose", true);
	gripperGoalClient = std::make_unique<actionlib::SimpleActionClient<handling_msgs::GoapMsgAction>> ("/dexterity/gripper", true);
	inspectGoalClient = std::make_unique<actionlib::SimpleActionClient<handling_msgs::GoapMsgAction>> ("inspect_action", true);
	extractGoalClient = std::make_unique<actionlib::SimpleActionClient<handling_msgs::GoapMsgAction>> ("extract_action", true);
	graspGoalClient = std::make_unique<actionlib::SimpleActionClient<handling_msgs::GoapMsgAction>> ("grab_action", true);
	dropGoalClient = std::make_unique<actionlib::SimpleActionClient<handling_msgs::GoapMsgAction>> ("/dexterity/drop", true);
	timer = new QTimer (this);
	connect (timer, SIGNAL(timeout()), this, SLOT(update()));

	connect (resetBtn, &QPushButton::clicked, this, &RobotArmControlWidget::slotBtnResetClicked);
	connect (getCurrentPose, &QPushButton::clicked, this, &RobotArmControlWidget::slotBtnCurrentPoseClicked);
	connect (clearOctomap, &QPushButton::clicked, this, &RobotArmControlWidget::slotBtnClearOctomapClicked);
	connect (resetArmWrist, &QPushButton::clicked, this, &RobotArmControlWidget::slotBtnWristResetClicked);
	connect (openGripper, &QPushButton::clicked, this, &RobotArmControlWidget::slotBtnGripperOpenClicked);
	connect (closeGripper, &QPushButton::clicked, this, &RobotArmControlWidget::slotBtnGripperCloseClicked);
	connect (execute, &QPushButton::clicked, this, &RobotArmControlWidget::slotBtnExecuteClicked);
	connect (taskListWidget, &QListWidget::clicked, this, &RobotArmControlWidget::slotTaskListWidgetClicked);
	connect (manualMode, &QRadioButton::clicked, this, &RobotArmControlWidget::slotChangeControlMode);
	connect (planningMode, &QRadioButton::clicked, this, &RobotArmControlWidget::slotChangeControlMode);
	connect (cancelCurrentAction, &QPushButton::clicked, this, &RobotArmControlWidget::slotBtnCancelClicked);
	connect (robotFrame, &QRadioButton::clicked, this, &RobotArmControlWidget::slotChangeToRobotRefFrame);
	connect (cameraFrame, &QRadioButton::clicked, this, &RobotArmControlWidget::slotChangeToCameraRefFrame);
	connect (autoControl, &QRadioButton::clicked, this, &RobotArmControlWidget::slotChangeToAutoMode);
	connect (joycontrol, &QRadioButton::clicked, this, &RobotArmControlWidget::slotChangeToJoyMode);

}

RobotArmControlWidget::~RobotArmControlWidget ()
{

}

QWidget* RobotArmControlWidget::createInstance ()
{
	return new RobotArmControlWidget;
}

void RobotArmControlWidget::init ()
{
	addSubscriber (goHome_sub, "/dexterity/goHome/feedback", 1, &RobotArmControlWidget::goHomeCallBack, this);
	addSubscriber (grab_sub, "/grab_action/feedback", 1, &RobotArmControlWidget::grabCallBack, this);
	addSubscriber (extract_sub, "/extract_action/feedback", 1, &RobotArmControlWidget::extractCallBack, this);
	addSubscriber (inspect_sub, "/inspect_action/feedback", 1, &RobotArmControlWidget::inspectCallBack, this);
	addSubscriber (poseGoal_sub, "/dexterity/goalPose/feedback", 1, &RobotArmControlWidget::poseGoalCallBack, this);
	addSubscriber (drop_sub, "/dexterity/drop/feedback", 1, &RobotArmControlWidget::dropCallBack, this);
	addSubscriber (object_grabbed_grasp, "/dexterity/object_grabbed_grasp", 1, &RobotArmControlWidget::objectGrabbedGraspCB, this);
	addSubscriber (object_grabbed_extract, "/dexterity/object_grabbed_extract", 1, &RobotArmControlWidget::objectGrabbedExtractCB, this);
	addPublisher<handling_msgs::JoyControlArmWristReset> (wristReset_sub, "/dexterity/arm_wrist_reset", 1);
	addPublisher<handling_msgs::JoyControlArmWristReset> (Reset_sub, "/dexterity/arm_reset", 1);
	addServiceClient<handling_msgs::GetCurrentPose> (currentPoseService, "/dexterity/get_current_pose_endeffector");
	addServiceClient<std_srvs::Empty> (clearOctomapService, "clear_octomap");
	addServiceServer (toggleSrv, "/dexterity/GUI/toggle_exe_cancel", &RobotArmControlWidget::toggleExeCancelBtnCB, this);
	timer->start (1000);
}

void RobotArmControlWidget::update ()
{
	std::string temp, exeMode;

	getNodeHandle ()->getParam ("/init/current_joy_mode", temp);

	if (temp == "Manual")
	{
		if (not manualMode->isChecked ())
		{
			manualMode->setProperty ("checked", true);
		}
	}
	if (temp == "Planning")
	{
		if (not planningMode->isChecked ())
		{
			planningMode->setProperty ("checked", true);
		}
	}

	getNodeHandle ()->getParam ("/init/current_execution_mode", exeMode);

	if (exeMode == automatic)
	{
		manualMode->setProperty ("enabled", false);
		planningMode->setProperty ("enabled", false);
		joycontrol->setProperty ("enabled", false);
		autoControl->setProperty ("enabled", true);
		autoControl->setProperty ("checked", true);
		robotFrame->setProperty ("enabled", false);
		cameraFrame->setProperty ("enabled", false);
	}
	else if (exeMode == joystick)
	{
		autoControl->setProperty ("enabled", false);
		manualMode->setProperty ("enabled", true);
		planningMode->setProperty ("enabled", true);
		joycontrol->setProperty ("enabled", true);
		joycontrol->setProperty ("checked", true);
		robotFrame->setProperty ("enabled", true);
		cameraFrame->setProperty ("enabled", true);
	}

	getNodeHandle ()->getParam ("/init/reference_frames/current_ref_frame", temp);

	if (temp == "GETjag/base_link")
	{

		robotFrame->setProperty ("checked", true);
		cameraFrame->setProperty ("checked", false);

	}
	if (temp == "GETjag/arm_wrist_roll_link")
	{

		robotFrame->setProperty ("checked", false);
		cameraFrame->setProperty ("checked", true);

	}

	if (not joycontrol->isChecked ())
	{
		manualMode->setProperty ("enabled", false);
		planningMode->setProperty ("enabled", false);
	}

}

void RobotArmControlWidget::poseGoalCallBack (const handling_msgs::GoapMsgActionFeedbackConstPtr& feedback)
{
	if (not isPoseGoalExecuted)
		return;
	progressBarCB (feedback->feedback.progress);

}
void RobotArmControlWidget::goHomeCallBack (const handling_msgs::GoapMsgActionFeedbackConstPtr& feedback)
{
	progressBarCB (feedback->feedback.progress);
}

void RobotArmControlWidget::grabCallBack (const handling_msgs::GoapMsgActionFeedbackConstPtr& feedback)
{
	progressBarCB (feedback->feedback.progress);

}

void RobotArmControlWidget::extractCallBack (const handling_msgs::GoapMsgActionFeedbackConstPtr& feedback)
{
	progressBarCB (feedback->feedback.progress);
}

void RobotArmControlWidget::inspectCallBack (const handling_msgs::GoapMsgActionFeedbackConstPtr& feedback)
{
	progressBarCB (feedback->feedback.progress);

}

void RobotArmControlWidget::dropCallBack (const handling_msgs::GoapMsgActionFeedbackConstPtr& feedback)
{
	progressBarCB (feedback->feedback.progress);
}

void RobotArmControlWidget::progressBarCB (double fb)
{
	try
	{
		std::string temp;
		getNodeHandle ()->getParam ("currentActionStatus", temp);
		if (fb == -1)
		{
			Q_EMIT progressBar->setFormat (QString::fromStdString (temp));
			progressBar->setAlignment (Qt::AlignCenter);
			return;
		}
		Q_EMIT progressBar->setValue (fb);
		progressBar->setTextVisible (true);
		Q_EMIT progressBar->setFormat (QString::fromStdString (temp));
		progressBar->setAlignment (Qt::AlignCenter);

	}
	catch (std::exception &e)
	{
		qFatal ("Error %s sending event %s to object %s (%s)", e.what ());
	}
}

void RobotArmControlWidget::refresh ()
{

}

void RobotArmControlWidget::reset ()
{

}

bool RobotArmControlWidget::toggleExeCancelBtnCB (handling_msgs::ToggleGUIExecuteCancelBtn::Request &req, handling_msgs::ToggleGUIExecuteCancelBtn::Response &res)
{
	if (req.enableExecuteBtn)
	{
		execute->setProperty ("enabled", true);
		res.isDone = true;
	}

	return true;
}

void RobotArmControlWidget::objectGrabbedGraspCB (const handling_msgs::ObjectGrabbedConstPtr& msg)
{
	try
	{
		if (msg->isGrabbed == true)
		{
			auto currentFlags = taskListWidget->item (1)->flags ();
			taskListWidget->item (1)->setFlags (currentFlags | Qt::ItemIsEnabled);
		}
	}
	catch (std::exception &e)
	{
		qFatal ("Error %s sending event %s to object %s (%s)", e.what ());
	}
}

void RobotArmControlWidget::objectGrabbedExtractCB (const handling_msgs::ObjectGrabbedConstPtr& msg)
{
	try
	{
		if (msg->isGrabbed == true)
		{
			auto currentFlags = taskListWidget->item (1)->flags ();
			taskListWidget->item (1)->setFlags (currentFlags | Qt::ItemIsEnabled);
		}
	}
	catch (std::exception &e)
	{
		qFatal ("Error %s sending event %s to object %s (%s)", e.what ());
	}
}

void RobotArmControlWidget::slotBtnWristResetClicked ()
{
	std_msgs::Bool msg;
	msg.data = true;
	publish (wristReset_sub, msg, true);
}

void RobotArmControlWidget::slotBtnCurrentPoseClicked ()
{
	handling_msgs::GetCurrentPose poseRes;
	poseRes.request.reqCurrentPose = true;
	callService (currentPoseService, poseRes, true);

	geometry_msgs::Pose pose = poseRes.response.pose;

	xSpinBox->setValue (pose.position.x);
	ySpinBox->setValue (pose.position.y);
	zSpinBox->setValue (pose.position.z);
	rollSpinBox->setValue (poseRes.response.rpy[0]);
	pitchSpinBox->setValue (poseRes.response.rpy[1]);
	yawSpinBox->setValue (poseRes.response.rpy[2]);
}

void RobotArmControlWidget::slotBtnClearOctomapClicked ()
{
	std_srvs::Empty emptyReq;
	callService (clearOctomapService, emptyReq, true);
}

void RobotArmControlWidget::slotBtnResetClicked ()
{
	if (planningMode->isChecked ())
	{
		handling_msgs::GoapMsgAction goal;
		goal.action_goal.goal.input = "{ goHome : true}";
		goHomeClient->sendGoal (goal.action_goal.goal);
	}
	else if (manualMode->isChecked ())
	{
		std_msgs::Bool msg;
		msg.data = true;
		publish (Reset_sub, msg, true);
	}
}

void RobotArmControlWidget::removeAllListWidgetItems ()
{
	for (int i = selectWhichTask->count (); i >= 0; --i)
	{
		selectWhichTask->removeItem (i);
	}

	for (int i = selectObject->count (); i >= 0; --i)
	{
		selectObject->removeItem (i);
	}
}

std::tuple<std::vector<std::string>, std::vector<std::string>> RobotArmControlWidget::loadParamsFromFile (const std::string yamlFileName)
{
	std::string path = ros::package::getPath ("arm_planning") + "/config/" + yamlFileName;
	YAML::Node root;
	try
	{
		root = YAML::LoadFile (path);
	}
	catch (const std::exception& e)
	{
		std::cout << e.what () << "\n";
	}
	int i = 0;

	std::vector<std::string> temp1;
	std::vector<std::string> temp2;
	for (YAML::const_iterator it = root.begin (); it != root.end (); ++it)
	{
		temp1.push_back (it->first.as<std::string> ());
		for (YAML::const_iterator it2 = it->second.begin (); it2 != it->second.end (); ++it2)
		{
			temp2.push_back (it2->first.as<std::string> ());
		}

	}
	return std::make_tuple (temp1, temp2);
}

void RobotArmControlWidget::slotBtnGripperOpenClicked ()
{
	handling_msgs::GoapMsgAction goal;
	std::string goalString = "{ gripper : open}";
	goal.action_goal.goal.input = goalString;
	gripperGoalClient->sendGoal (goal.action_goal.goal);
}

void RobotArmControlWidget::slotBtnGripperCloseClicked ()
{

	handling_msgs::GoapMsgAction goal;
	std::string goalString = "{ gripper : close}";
	goal.action_goal.goal.input = goalString;
	gripperGoalClient->sendGoal (goal.action_goal.goal);
}

void RobotArmControlWidget::addComboBoxItems (const std::string filename)
{
	QStringList objectsList, tasksList;
	currentAction = filename;
	auto temp = loadParamsFromFile (filename + ".yaml");
	auto objects = std::get<0> (temp);
	auto grasps = std::get<1> (temp);
	for (auto object : objects)
	{
		objectsList.append (QString::fromStdString (object));

	}
	defaultListOption = "Select " + currentAction + " points";
	tasksList.append (QString::fromStdString (defaultListOption));

	tasksList.append (QString::fromStdString ("random"));

	for (auto grasp : grasps)
	{
		tasksList.append (QString::fromStdString (grasp));
	}
	selectWhichTask->addItems (tasksList);
	selectObject->addItems (objectsList);
}

void RobotArmControlWidget::slotTaskListWidgetClicked ()
{
	removeAllListWidgetItems ();
	if (taskListWidget->selectedItems ().front ()->text ().toStdString () == "Grasp")
	{
		addComboBoxItems ("grasps");
	}
	else if (taskListWidget->selectedItems ().front ()->text ().toStdString () == "Inspect")
	{
		addComboBoxItems ("inspection");

	}
	else if (taskListWidget->selectedItems ().front ()->text ().toStdString () == "Extract")
	{
		addComboBoxItems ("extraction");
	}

}

std::string RobotArmControlWidget::convertGoalToGoapMsg (std::string task, const std::string objectID)
{
	if (task == defaultListOption or task == "any")
	{
		task = "None";
	}
	std::string x = xSpinBox->text ().toStdString ();
	std::string y = ySpinBox->text ().toStdString ();
	std::string z = zSpinBox->text ().toStdString ();
	std::replace (x.begin (), x.end (), ',', '.');
	std::replace (y.begin (), y.end (), ',', '.');
	std::replace (z.begin (), z.end (), ',', '.');

	std::string goapMsg = "{$activeObject.atPos : [" + x + "," + y + "," + z + ",0,0,0,0]" + "," + "$activeObject.id : " + objectID + ", $activeObject.which : " + task + "}";
	std::cout << goapMsg << std::endl;
	return goapMsg;
}

std::string RobotArmControlWidget::convertGoalToPoseGoal (tf2::Quaternion orientation)
{
	auto Ox = std::to_string (-orientation.x ());
	auto Oy = std::to_string (orientation.y ());
	auto Oz = std::to_string (orientation.z ());
	auto Ow = std::to_string (orientation.w ());

	std::string x = xSpinBox->text ().toStdString ();
	std::string y = ySpinBox->text ().toStdString ();
	std::string z = zSpinBox->text ().toStdString ();
	std::replace (x.begin (), x.end (), ',', '.');
	std::replace (y.begin (), y.end (), ',', '.');
	std::replace (z.begin (), z.end (), ',', '.');
	std::replace (Ox.begin (), Ox.end (), ',', '.');
	std::replace (Oy.begin (), Oy.end (), ',', '.');
	std::replace (Oz.begin (), Oz.end (), ',', '.');
	std::replace (Ow.begin (), Ow.end (), ',', '.');

	std::string goapMsg = "{$activeObject.atPos : [" + x + "," + y + "," + z + "," + Ox + "," + Oy + "," + Oz + "," + Ow + "]" + "}";
	return goapMsg;
}

void RobotArmControlWidget::slotBtnCancelClicked ()
{
	if (not (currentActionClient == nullptr))
	{
		currentActionClient->cancelGoal ();
		cancelCurrentAction->setProperty ("enabled", false);
	}
}

void RobotArmControlWidget::slotBtnExecuteClicked ()
{
	isPoseGoalExecuted = false;

	progressBar->setTextVisible (false);
	Q_EMIT progressBar->setValue (0);

	handling_msgs::GoapMsgAction goal;
	std::string goalString = convertGoalToGoapMsg (selectWhichTask->currentText ().toStdString (), selectObject->currentText ().toStdString ());
	goal.action_goal.goal.input = goalString;

	if (taskListWidget->selectedItems ().front ()->text ().toStdString () == "Grasp")
	{
		graspGoalClient->sendGoal (goal.action_goal.goal);

		currentActionClient = graspGoalClient;
	}
	else if (taskListWidget->selectedItems ().front ()->text ().toStdString () == "Inspect")
	{
		currentActionClient = inspectGoalClient;

		inspectGoalClient->sendGoal (goal.action_goal.goal);
	}
	else if (taskListWidget->selectedItems ().front ()->text ().toStdString () == "Extract")
	{
		currentActionClient = extractGoalClient;

		extractGoalClient->sendGoal (goal.action_goal.goal);
	}
	else if (taskListWidget->selectedItems ().front ()->text ().toStdString () == "Move Arm To Pose")
	{
		isPoseGoalExecuted = true;
		currentActionClient = poseGoalClient;

		handling_msgs::GoapMsgAction goal;
		tf2::Quaternion orientation;
		orientation.setRPY (-rollSpinBox->value (), pitchSpinBox->value (), yawSpinBox->value ());
		std::string goalString;
		goalString = convertGoalToPoseGoal (orientation);
		goal.action_goal.goal.input = goalString;

		poseGoalClient->sendGoal (goal.action_goal.goal);
	}
	else if (taskListWidget->selectedItems ().front ()->text ().toStdString () == "Drop")
	{
		currentActionClient = dropGoalClient;

		handling_msgs::GoapMsgAction goal;
		goal.action_goal.goal.input = "{drop : true }";

		dropGoalClient->sendGoal (goal.action_goal.goal);

		auto currentFlags = taskListWidget->item (1)->flags ();
		taskListWidget->item (1)->setFlags (currentFlags & ~Qt::ItemIsEnabled);
		taskListWidget->item (0)->setSelected (true);
	}

	execute->setProperty ("enabled", false);
	if (not cancelCurrentAction->isEnabled ())
		cancelCurrentAction->setProperty ("enabled", true);

}

void RobotArmControlWidget::slotChangeToAutoMode ()
{
	joycontrol->setProperty ("checked", false);

	manualMode->setProperty ("enabled", false);
	planningMode->setProperty ("enabled", false);

	getNodeHandle ()->setParam ("/init/current_execution_mode", automatic);

}

void RobotArmControlWidget::slotChangeToJoyMode ()
{
	if (not manualMode->isChecked ())
	{
		manualMode->setProperty ("enabled", false);
		planningMode->setProperty ("enabled", false);
	}

	autoControl->setProperty ("checked", false);

	manualMode->setProperty ("enabled", true);
	planningMode->setProperty ("enabled", true);

	getNodeHandle ()->setParam ("/init/current_execution_mode", joystick);
}

void RobotArmControlWidget::slotChangeControlMode ()
{
	if (manualMode->isChecked ())
		getNodeHandle ()->setParam ("/init/current_joy_mode", "Manual");
	else if (planningMode->isChecked ())
		getNodeHandle ()->setParam ("/init/current_joy_mode", "Planning");
}

void RobotArmControlWidget::slotChangeToRobotRefFrame ()
{
	if (robotFrame->isChecked ())
	{
		cameraFrame->setProperty ("checked", false);
		getNodeHandle ()->setParam ("/init/reference_frames/current_ref_frame", "GETjag/base_link");
	}
}

void RobotArmControlWidget::slotChangeToCameraRefFrame ()
{
	if (cameraFrame->isChecked ())
	{
		robotFrame->setProperty ("checked", false);
		getNodeHandle ()->setParam ("/init/reference_frames/current_ref_frame", "GETjag/arm_wrist_roll_link");
	}
}

