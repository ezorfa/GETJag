#include <dynamixel_msgs/JointState.h>

#include <arm_planning/JoyControl.h>

JoyControl::JoyControl (ros::NodeHandle& nh, std::shared_ptr<ArmControlParameterList>& paramList, std::shared_ptr<Executor>& exe, std::shared_ptr<Planning>& plan, std::shared_ptr<JointStateSubscriber>& jointStates)
		: exe (exe), plan (plan), jointState (jointStates), paramList (paramList)
{

	jointNames = paramList->getAllJoints ();
	staticParams = paramList->getStaticParams ();
	joyMappings = paramList->getJoyMappings ();
	controllerMap = paramList->getControllers ();

	goHomeClient = std::make_unique<actionlib::SimpleActionClient<handling_msgs::GoapMsgAction>> (staticParams.goHomeActionName, true);

	ros::Subscriber sub = nh.subscribe (staticParams.wristReset, 1, &JoyControl::armWristReset, this);
	ros::Subscriber sub2 = nh.subscribe (staticParams.reset, 1, &JoyControl::reset, this);
	// initial reset, the joints move to zero positions when this node is started.
	for (auto& joint : jointNames)
	{
		setSpeedService[joint] = nh.serviceClient<dynamixel_controllers::SetSpeed> (controllerMap[joint] + staticParams.speedTopic);
	}
	joySub = nh.subscribe<sensor_msgs::Joy> ("joy", 10, &JoyControl::joySubCB, this);
	reset ();
	paramList->setJoystickControlMode (ArmControlParameterList::JoyControlMode::PLANNING);
	paramList->setRefFrame (staticParams.baseLinkRefFrame);

	ros::Rate rate (20);

	while (ros::ok ())
	{
		ros::spinOnce ();

		if (not paramList->getExecutionMode () == ArmControlParameterList::ExecutionMode::Joystick)
		{
			resetTarget = true;
			for (auto& joint : jointNames)
			{
				targetJointAngles[joint] = jointState->getCurrentState (joint).current_pos;
			}
			continue;
		}
		auto currentMode = paramList->getJoystickControlMode ();
		// Planning mode
		if (currentMode == ArmControlParameterList::JoyControlMode::PLANNING)
		{
			// This flag is used to check if there is a need to plan for next increments.
			bool updateValue = false;

			for (auto const& itr : joyIncrements)
			{
				if (itr.second != 0)
				{
					updateValue = true;
					break;
				}
			}
			// check if pitch, roll, yaw is changed. If so reset the targetPose in the planning node.
			if (diffPitch != 0 or diffRoll != 0 or diffYaw != 0)
			{
				resetTarget = true;
				manualControl ();
			}
			// exe->checkIfGoalPosReached () is used to check if the joint values reached the goal position. If so,
			// then, plan for next increments.
			if (updateValue and (exe->checkIfGoalPosReached () or resetTarget))
			{
				bool success = plan->plan (joyIncrements, paramList->getRefFrame (), targetJointAngles, resetTarget);
				if (success)
				{
					resetTarget = false;
					exe->execute (targetJointAngles);

				}
			}
		}
		// Manual mode
		else if (currentMode == ArmControlParameterList::JoyControlMode::MANUAL)
		{
			manualControl ();
		}
		rate.sleep ();
	}
}

// This function is called if there are any increments to one/many specific joints.
void JoyControl::manualControl ()
{
	targetJointAngles["GETjag/arm_base_joint"] += diffBase;
	targetJointAngles["GETjag/arm_shoulder_joint"] += diffShoulder;
	targetJointAngles["GETjag/arm_elbow_joint"] += diffElbow;
	targetJointAngles["GETjag/arm_wrist_yaw_joint"] += diffYaw;
	targetJointAngles["GETjag/arm_wrist_pitch_joint"] += diffPitch;
	targetJointAngles["GETjag/arm_wrist_roll_joint"] += diffRoll;
	targetJointAngles["GETjag/arm_gripper_joint"] += diffGripper;

	for (auto& joint : jointNames)
	{
		setSpeed (joint, staticParams.manualJointSpeeds);
		exe->execute (joint, targetJointAngles[joint]);
	}
}

void JoyControl::reset (const handling_msgs::JoyControlArmWristResetConstPtr& ptr)
{
	exe->abortExecution ();
	paramList->setRefFrame (staticParams.baseLinkRefFrame);
	resetTarget = true;
	paramList->setExecutionMode (ArmControlParameterList::ExecutionMode::Joystick);
	for (auto const& joint : jointNames)
	{
		targetJointAngles[joint] = 0.0;
	}
	manualControl ();
}

void JoyControl::setSpeed (const std::string joint, const double speed)
{
	dynamixel_controllers::SetSpeed speedData;
	speedData.request.speed = speed;
	setSpeedService[joint].call (speedData);
}

void JoyControl::joySubCB (const sensor_msgs::JoyConstPtr &joy)
{

	if (not paramList->getExecutionMode () == ArmControlParameterList::ExecutionMode::Joystick)
	{
		resetTarget = true;
		for (auto& joint : jointNames)
		{
			targetJointAngles[joint] = jointState->getCurrentState (joint).current_pos;
		}
		return;
	}

	auto currentJoyMode = paramList->getJoystickControlMode ();
	double btnPlanning = joy->buttons[joyMappings.BTN_PLANNING];
	double btnManual = joy->buttons[joyMappings.BTN_MANUAL];

	if (currentJoyMode == ArmControlParameterList::JoyControlMode::PLANNING)
	{
		if (paramList->getRefFrame () == staticParams.baseLinkRefFrame)
		{
			joyIncrements["y"] = -(staticParams.maxScalingFactorJoyPlanMode) * (double)joy->axes[joyMappings.AXIS_Y];
			joyIncrements["z"] = (staticParams.maxScalingFactorJoyPlanMode) * (double)joy->axes[joyMappings.AXIS_Z];
		}
		else
		{
			joyIncrements["y"] = (staticParams.maxScalingFactorJoyPlanMode) * (double)joy->axes[joyMappings.AXIS_Z];
			joyIncrements["z"] = -(staticParams.maxScalingFactorJoyPlanMode) * (double)joy->axes[joyMappings.AXIS_Y];
		}

		joyIncrements["x"] = (staticParams.maxScalingFactorJoyPlanMode) * (double)joy->axes[joyMappings.AXIS_X];

		diffPitch = -staticParams.maxPitchScalingFactorManualMode * (double)joy->axes[joyMappings.AXIS_PITCH];
		diffRoll = -staticParams.maxRollScalingFactorManualMode * (double)joy->axes[joyMappings.AXIS_ROLL];
		diffYaw = staticParams.maxYawScalingFactorManualMode * (double)joy->axes[joyMappings.AXIS_YAW];

//		{
//			bool updateValue = false;
//
//			for (auto const& itr : joyIncrements)
//			{
//				if (itr.second != 0)
//				{
//					updateValue = true;
//					break;
//				}
//			}
//
//			if (diffPitch != 0 or diffRoll != 0 or diffYaw != 0)
//			{
//				resetTarget = true;
//				manualControl ();
//			}
//
//			if (updateValue and (exe->checkIfGoalPosReached () or resetTarget))
//			{
//				bool success = plan->plan (joyIncrements, paramList->getRefFrame (), targetJointAngles, resetTarget);
//				if (success)
//				{
//					resetTarget = false;
//					exe->execute (targetJointAngles);
//
//				}
//			}
//		}
	}
	if (currentJoyMode == ArmControlParameterList::JoyControlMode::MANUAL)
	{
		diffShoulder = staticParams.maxShoulderScalingFactorManualMode * (double)joy->axes[joyMappings.AXIS_SHOULDER];
		diffElbow = staticParams.maxElbowScalingFactorManualMode * (double)joy->axes[joyMappings.AXIS_ELBOW];
		diffBase = staticParams.maxBaseScalingFactorManualMode * (double)joy->axes[joyMappings.AXIS_BASE];
		diffPitch = -staticParams.maxPitchScalingFactorManualMode * (double)joy->axes[joyMappings.AXIS_PITCH];
		diffRoll = -staticParams.maxRollScalingFactorManualMode * (double)joy->axes[joyMappings.AXIS_ROLL];
		diffYaw = staticParams.maxYawScalingFactorManualMode * (double)joy->axes[joyMappings.AXIS_YAW];

//		manualControl ();
	}

	if (joy->buttons[joyMappings.BTN_RESET] > ACTIVE_ZONE and currentJoyMode == ArmControlParameterList::JoyControlMode::MANUAL)
	{
		ROS_INFO_STREAM("Reset!");
		reset ();
	}

	if (btnPlanning > ACTIVE_ZONE and (currentJoyMode != ArmControlParameterList::JoyControlMode::PLANNING))
	{
		paramList->setExecutionMode (ArmControlParameterList::ExecutionMode::Joystick);
		// set the global joints angles to the current pose.
		for (auto& joint : jointNames)
		{
			targetJointAngles[joint] = jointState->getCurrentState (joint).current_pos;
		}
		paramList->setJoystickControlMode (ArmControlParameterList::JoyControlMode::PLANNING);
		resetTarget = true;
		ROS_INFO_STREAM("JoyStick Control: In Planning mode");
	}

	if (btnManual > ACTIVE_ZONE and currentJoyMode != ArmControlParameterList::JoyControlMode::MANUAL)
	{
		paramList->setExecutionMode (ArmControlParameterList::ExecutionMode::Joystick);
		for (auto& joint : jointNames)
		{
			targetJointAngles[joint] = jointState->getCurrentState (joint).current_pos;
		}
		paramList->setJoystickControlMode (ArmControlParameterList::JoyControlMode::MANUAL);
		ROS_INFO_STREAM("JoyStick Control: In manual mode");
	}

	if (joy->buttons[joyMappings.BTN_BASELINK] > ACTIVE_ZONE)
	{
		resetTarget = true;
		ROS_INFO_STREAM(staticParams.baseLinkRefFrame);
		paramList->setRefFrame (staticParams.baseLinkRefFrame);
	}
	if (joy->buttons[joyMappings.BTN_EEF] > ACTIVE_ZONE)
	{
		resetTarget = true;
		ROS_INFO_STREAM(staticParams.endEffectorRefFrame);
		paramList->setRefFrame (staticParams.endEffectorRefFrame);
	}

	if (joy->buttons[joyMappings.BTN_RESET] > ACTIVE_ZONE and currentJoyMode == ArmControlParameterList::JoyControlMode::PLANNING)
	{
		handling_msgs::GoapMsgAction goal;
		goal.action_goal.goal.input = "{goHome : true }";
		goHomeClient->sendGoal (goal.action_goal.goal);
	}

	if (joy->buttons[joyMappings.BTN_WRIST_RESET] > ACTIVE_ZONE)
	{
		armWristReset ();
	}

	if (joy->buttons[joyMappings.BTN_G_CLOSE] > ACTIVE_ZONE)
	{
		targetJointAngles[staticParams.gripper] = 0;

		if (currentJoyMode == ArmControlParameterList::JoyControlMode::MANUAL)
		{
			manualControl ();
		}
		else
			exe->setupGripperControl (ArmControlParameterList::CLOSE, 0);

		paramList->setGripperMode (ArmControlParameterList::CLOSE);

	}
	if (joy->buttons[joyMappings.BTN_G_OPEN] > ACTIVE_ZONE)
	{
		targetJointAngles[staticParams.gripper] = staticParams.gripperOpenAngle;

		if (currentJoyMode == ArmControlParameterList::JoyControlMode::MANUAL)
		{
			manualControl ();
		}
		else
			exe->setupGripperControl (ArmControlParameterList::OPEN, staticParams.gripperOpenAngle);

		paramList->setGripperMode (ArmControlParameterList::OPEN);
	}

}

// A useful function to reset only the wirst portion of the arm. Helpful in cases of arm getting stuck at wierd locations.
void JoyControl::armWristReset (const handling_msgs::JoyControlArmWristResetConstPtr& ptr)
{
	ROS_INFO_STREAM("WRIST RESET!!");
	resetTarget = true;
	auto joints = paramList->getWristJoints ();
	for (auto joint : joints)
	{
		targetJointAngles[joint] = 0.0;
	}
	manualControl ();
}

