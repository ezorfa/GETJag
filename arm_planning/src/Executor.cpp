#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <iostream>
#include <xmlrpcpp/XmlRpcValue.h>

#include <ros/duration.h>
#include <ros/init.h>
#include <rosconsole/macros_generated.h>
#include <std_msgs/Float64.h>

#include <arm_planning/Executor.h>

Executor::Executor (ros::NodeHandle& nh, std::shared_ptr<ArmControlParameterList>& paramList, std::shared_ptr<JointStateSubscriber>& jointStates)
		: jointState (jointStates), paramList (paramList)
{
	jointNames = paramList->getAllJoints ();
	controllerMap = paramList->getControllers ();
	staticParams = paramList->getStaticParams ();
	maxJointSpeedLimits = paramList->getJointSpeedLimits ();

	for (auto& joint : jointNames)
	{
		controllerPublisher[joint] = nh.advertise<std_msgs::Float64> (controllerMap[joint] + staticParams.commandTopic, 10);
		setSpeedService[joint] = nh.serviceClient<dynamixel_controllers::SetSpeed> (controllerMap[joint] + staticParams.speedTopic);
		ROS_INFO_STREAM(controllerMap[joint] + staticParams.commandTopic);
	}

	gripperMode = ArmControlParameterList::GripperMode::CLOSE;
	jointStateSub = nh.subscribe<sensor_msgs::JointState> (staticParams.jointStatesTopic, 1, &Executor::execute, this);
	grabingTimer = nh.createTimer (ros::Duration (0.2), &Executor::gripperTimerCallback, this);
	grabingTimer.stop ();
}

void Executor::execute (const sensor_msgs::JointStateConstPtr& jointState)
{
	if (not hasActiveGoal)
		return;

	loopCount++;
	if (paramList->getExecutionMode () != ArmControlParameterList::ExecutionMode::Joystick && loopCount > staticParams.executionTimeOutCount)
	{
		ROS_INFO_STREAM("TIME OUT!");
		abortExecution ();
		if (server != nullptr)
			server->setPreempted ();
		loopCount = 0;
		return;
	}

	for (std::size_t i = 0; i < jointState->name.size (); ++i)
	{
		currentState[jointState->name[i]] = jointState->position[i];
	}

	int i = 0;
	unsigned int noErrorCount = 0;
	for (auto& joint : jointNames)
	{
		if (joint != staticParams.gripper)
		{
			if (paramList->getExecutionMode () == ArmControlParameterList::ExecutionMode::Joystick)
			{
				goalWayPoint[joint] = targetJointAngles[joint];
			}
			else
			{
				goalWayPoint[joint] = traj.points[currentWayPoint].positions[i];
			}

			double error = std::abs (goalWayPoint[joint] - currentState[joint]);

			if (paramList->getExecutionMode () == ArmControlParameterList::ExecutionMode::Joystick)
			{
				if (error < staticParams.joyModeErrorMargin)
				{
					noErrorCount++;
				}
				else
					isNearGoalPos = false;
			}
			else
			{
				if (error < staticParams.autoModeErrorMargin)
				{
					noErrorCount++;
				}
			}
		}
		i++;
	}
	if (noErrorCount == jointNames.size () - 1)
	{
		loopCount = 0;
		if (paramList->getExecutionMode () == ArmControlParameterList::ExecutionMode::Joystick)
			isNearGoalPos = true;
		else
		{
			auto percent = ((double)currentWayPoint / (double)traj.points.size ()) * 100;
			actionServer->setFeedback (percent);

			if (currentWayPoint == traj.points.size () - 1)
			{
				hasActiveGoal = false;
				if (actionServer != nullptr)
				{
					actionServer->setSucceeded ();
					actionServer = nullptr;
				}
				return;
			}

			currentWayPoint++;

			int j = 0;
			for (auto& joint : jointNames)
			{
				if (joint != staticParams.gripper)
				{
					std_msgs::Float64 state;
					state.data = traj.points[currentWayPoint].positions[j];
					double speed = std::abs (traj.points[currentWayPoint].velocities[j]);
					if (speed < staticParams.lowerSpeedLimit)
					{
						speed = staticParams.lowerSpeedLimit;
					}
					speedData.request.speed = speed;
					if (joint == "GETjag/arm_base_joint")
					{
						speedData.request.speed = speed * 2;
					}
					setSpeedService[joint].call (speedData);
					controllerPublisher[joint].publish (state);
				}
				j++;
			}
		}
	}
	if (objectGrabbed)
	{
		monitorGrabStatus ();
	}
}

void Executor::execute (const std::string joint, const double value)
{
	std_msgs::Float64 state;
	state.data = value;
	//~ if (isJointOverloaded(joint, value)) return;
	controllerPublisher[joint].publish (state);
}

void Executor::execute (std::map<std::string, double>& jointAngles)
{
	targetJointAngles = jointAngles;
	calculateSpeed (jointAngles);
	// setting pitch and yaw speed maximum because we want the pitch of the end-effector to seemingly look in a fixed position
	// while moving with joystick.
	calculatedSpeed["GETjag/arm_wrist_pitch_joint"] = 1;
	calculatedSpeed["GETjag/arm_wrist_yaw_joint"] = 1;
	// speed for the arm base joint is doubled temporarily until the hardware is fixed, as its speed is
	// defaultly set to half its actual value.
	calculatedSpeed["GETjag/arm_base_joint"] = calculatedSpeed["GETjag/arm_base_joint"] * 2;
	for (auto& joint : jointNames)
	{
		if (joint != staticParams.gripper)
		{
			if (std::isnan (calculatedSpeed[joint]))
			{
				return;
			}
			speedData.request.speed = calculatedSpeed[joint] * staticParams.joyMaxVelocityScalingFactor;
			setSpeedService[joint].call (speedData);
			execute (joint, jointAngles[joint]);
		}
	}
	hasActiveGoal = true;
}

bool Executor::isJointOverloaded (std::string joint, const double value)
{
	if (std::abs (jointState->getCurrentState (joint).load) >= staticParams.maxGrabLoad)
	{
		if (jointState->getCurrentState (joint).load <= staticParams.maxGrabLoad)
		{
			return (value < jointState->getCurrentState (joint).current_pos) ? false : true;
		}
		else
		{
			return (value > jointState->getCurrentState (joint).current_pos) ? false : true;
		}
	}
	else
	{
		return false;
	}
}

bool Executor::checkIfGoalPosReached ()
{
	if (not isNearGoalPos)
		return false;

	return true;
}

void Executor::calculateSpeed (std::map<std::string, double>& jointAngles)
{
	std::map<std::string,double> dist;
	double maxTime;
	double tempTime;
	maxTime = 0;
	for (auto& joint : jointNames)
	{
		dist[joint] = std::abs (jointAngles[joint] - jointState->getCurrentState (joint).current_pos);
		tempTime = (double) dist[joint] / (double) maxJointSpeedLimits[joint];

		if (maxTime < tempTime)
		{
			maxTime = tempTime;
		}
	}

	for (auto& joint : jointNames)
	{
		calculatedSpeed[joint] = (double) dist[joint] / (double) maxTime;
		if (calculatedSpeed[joint] < staticParams.lowerSpeedLimit)
		{
			calculatedSpeed[joint] = staticParams.lowerSpeedLimit;
		}
	}
}

void Executor::setSpeed (const std::string joint, const double speed)
{
	dynamixel_controllers::SetSpeed speedData;
	speedData.request.speed = speed;
	setSpeedService[joint].call (speedData);
}

// This function is invoked if gripper is to be closed or opened. gripperAction param is optional.
void Executor::setupGripperControl (ArmControlParameterList::GripperMode mode, double pos, ActionServerInterface* gripServer)
{
	gripperServer = gripServer;
	gripperMode = mode;
	counterGrabingTimer = 0;
	gripperGoalPos = pos;
	setSpeed (staticParams.gripper, staticParams.manualJointSpeeds);
	execute (staticParams.gripper, gripperGoalPos);
	grabingTimer.start ();
}

void Executor::gripperTimerCallback (const ros::TimerEvent& event)
{
	double currentLoad;
	bool stopGrabbing = false;

	if (gripperMode == 0)
	{
		currentLoad = jointState->getCurrentState (staticParams.gripper).load;

		// If load exceeds certain threshold then stop the gripper.
		if (currentLoad > staticParams.maxGrabLoad)
		{
			stopGrabbing = true;
			objectGrabbed = true;
			ROS_INFO_STREAM("Object is Grabbed");
		}
		else
		{
			if (jointState->getCurrentState (staticParams.gripper).current_pos <= gripperGoalPos + 0.1)
			{
				stopGrabbing = true;
			}
		}
	}
	if (gripperMode == 1)
	{
		if (jointState->getCurrentState (staticParams.gripper).current_pos >= gripperGoalPos - 0.1)
		{
			stopGrabbing = true;
		}
	}

	if (not (stopGrabbing))
	{
		counterGrabingTimer += 1;
		if (counterGrabingTimer > staticParams.grabTimeOutCount)
		{
			stopGrabbing = true;
		}
	}

	if (stopGrabbing)
	{
		counterGrabingTimer = 0;
		grabingTimer.stop ();
		if (gripperServer != nullptr)
		{
			gripperServer->setSucceeded ();
			gripperServer = nullptr;
		}

		if (gripperMode == 1)
		{
			ROS_INFO("Gripper is opened");
			objectGrabbed = false;
		}
		else
		{
			ROS_INFO("Gripper is closed");
		}

	}
}

void Executor::abortExecution ()
{
	hasActiveGoal = false;
	if (actionServer != nullptr)
	{
		actionServer->setPreempted ();
	}
}

// This function is used to set ActionServerInterface to the server which called this functions so that functions like set succeeded can be invoked
// fromt this class.
void Executor::setServer (ActionServerInterface* s)
{
	if (server != nullptr)
	{
		if (server != s && server->isActive ())
		{
			ROS_INFO_STREAM("Preempting older server!!!");
			abortExecution ();
			if (actionServer != nullptr)
			{
				actionServer->setPreempted ();
			}
		}
	}

	server = s;
}

// This function sets the given or goal trajectory to the global variable traj.
void Executor::executePlan (moveit_msgs::RobotTrajectory trajectory, ActionServerInterface* server)
{
	actionServer = server;
	traj = trajectory.joint_trajectory;

	if (traj.points.size () > 1)
	{
		currentWayPoint = 0;
		hasActiveGoal = true;
	}
}

// This function is used to continuously monitor if the object is dropped while moving to some pose with object grabbed.
// If dropped, then the execution aborts.
void Executor::monitorGrabStatus ()
{
	if (currentJointLoad[staticParams.gripper] <= staticParams.maxGrabLoad)
	{
		ROS_INFO_STREAM("OBJECT IS DROPPED!");
		abortExecution ();
	}
}

// This function is used to check if the executor has some goal running.
bool Executor::isBusy ()
{
	return hasActiveGoal;
}

bool Executor::isObjectGrabbed ()
{
	return objectGrabbed;
}

