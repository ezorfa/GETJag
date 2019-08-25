/*
 * GraspAction.cpp
 *
 *  Created on: Mar 30, 2019
 *      Author: mohammed
 */

#include <ros/ros.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <eigen3/Eigen/Eigen>

int main (int argc, char** argv)
{
	ros::init (argc, argv, "panda_arm_pick_place");
	ros::NodeHandle nh;
	ros::AsyncSpinner spinner (1);
	spinner.start ();

	robot_model_loader::RobotModelLoader robot_model_loader ("robot_description");
	robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel ();
	ROS_INFO("Model frame: %s", kinematic_model->getModelFrame ().c_str ());
	robot_state::RobotStatePtr kinematic_state (new robot_state::RobotState (kinematic_model));
	double current_pos[] =
	{0, 0, 0, 0, 0, 0};

	double jointAngle;
	int limit = 0;
	ROS_INFO_STREAM("Enter values in the following order:"
			"base_joint, "
			"shoulder_joint, "
			"elbow_joint, "
			"yaw_joint, "
			"pitch_joint, "
			"roll_joint");
	while (std::cin >> jointAngle)
	{
		current_pos[limit] = jointAngle;
		if (limit == 5)
		{
			ROS_INFO("Joint pos = [ %f, %f, %f, %f, %f, %f ]", current_pos[0], current_pos[1], current_pos[2], current_pos[3], current_pos[4], current_pos[5]);
			kinematic_state->setVariablePositions (current_pos);
			kinematic_state->update ();
			const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform ("GETjag/arm_wrist_roll_link");
			ROS_INFO_STREAM("------------------------------------");
			ROS_INFO_STREAM("Translation: x		" << end_effector_state.translation().x());
			ROS_INFO_STREAM("Translation: y		" << end_effector_state.translation().y());
			ROS_INFO_STREAM("Translation: z		" << end_effector_state.translation().z());
			ROS_INFO_STREAM("------------------------------------");
			ROS_INFO_STREAM("Rotation: x		" << end_effector_state.translation().x());
			ROS_INFO_STREAM("Rotation: y		" << end_effector_state.translation().y());
			ROS_INFO_STREAM("Rotation: z		" << end_effector_state.translation().z());
			ROS_INFO_STREAM("Rotation: w		" << end_effector_state.translation().w());
			ROS_INFO_STREAM("------------------------------------");
			ROS_INFO_STREAM("Enter values in the following order:"
					"base_joint, "
					"shoulder_joint, "
					"elbow_joint, "
					"yaw_joint, "
					"pitch_joint, "
					"roll_joint");
			limit = -1;

		}
		limit++;

	}

	ros::waitForShutdown ();
	return 0;
}

