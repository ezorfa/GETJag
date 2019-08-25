/*
 * moveArm.h
 *
 *  Created on: Mar 2, 2019
 *      Author: mohammed
 */

#ifndef ROS_PG_WS_2018_19_ARM_PLANNING_SRC_MOVEARM_H_
#define ROS_PG_WS_2018_19_ARM_PLANNING_SRC_MOVEARM_H_

#include <ros/ros.h>
#include <dynamixel_msgs/JointState.h>
#include <string.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_msgs/String.h>

#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/frames_io.hpp>
#include <robot_arm_msgs/CurrentArmState.h>


#include <control_msgs/JointControllerState.h>
#include <sensor_msgs/JointState.h>

class RobotM
{
public:
		RobotM ();
		~RobotM ();

		KDL::Frame startFrame;
		std::vector<float>  startJoint; // home-position

		KDL::Frame moveTo(KDL::Frame l_cartTargetPos, float l_pitch_offset, bool l_axisMoveEnable);
		KDL::Frame moveTo(KDL::Frame l_cartTargetPos);

		float moveTo(std::string l_jointName, float l_targetPos);
		float moveTo(int l_jointNum, float l_targetPos);

		KDL::Frame aktCartesianPos;
		int numberOfJoints;
		std::vector<float> getActJointAngel();
		void setSpeed(int l_jointNum, float l_speed);
		void setDefSpeed();

		void resetJointAnglesForIK();

		int checkJointAngel(int l_jointNum, float l_targetPos);
		int checkJointAngel(std::string l_jointName, float l_targetPos);

		double getLoadJoint(int l_jointNum);
		double getLoadJoint(std::string l_jointName);


		bool getJag_simultion = true;
//		bool publish_aktRobotState_for_moveIt;
//		sensor_msgs::JointState jointState_for_moveIt;
//
//		ros::Publisher publisher_jointState_for_moveIt;

	private:

		struct struct_limits{
			float min;
			float max;
		};

		std::map<std::string, ros::Subscriber> motorSubscribers;
		std::map<std::string, ros::Publisher> motorPublishers;
		std::map<std::string, dynamixel_msgs::JointState> motorStates;

		std::vector<std::string> controllerList;
		std::map<std::string, float> maxLoad;

		std::map<std::string, float> maxVel;
		std::map<std::string, ros::ServiceClient> setSpeedService;

		std::map<std::string, struct_limits> jointLimits;

		void motorStateCallback (const dynamixel_msgs::JointStateConstPtr& msg);


		ros::ServiceServer serviceCurrentArmState;


		void getActCartPos();
		void setSpeed(std::string l_jointName, float l_speed);
		bool currentArmStateCallback(robot_arm_msgs::CurrentArmState::Request& request, robot_arm_msgs::CurrentArmState::Response& response);

};


#endif /* ROS_PG_WS_2018_19_ARM_PLANNING_SRC_MOVEARM_H_ */
