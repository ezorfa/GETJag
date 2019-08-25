#include <actionlib/server/simple_action_server.h>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <eigen3/Eigen/Eigen>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <ros/ros.h>

#include <arm_planning/Executor.h>
#include <arm_planning/JoyControl.h>
#include <arm_planning/Planning.h>
#include <arm_planning/GoHomeAction.h>
#include <arm_planning/PoseGoalAction.h>
#include <arm_planning/GrabAction.h>
#include <arm_planning/DropAction.h>
#include <arm_planning/InspectAction.h>
#include <arm_planning/JointStateSubscriber.h>
#include <arm_planning/ArmControlParameterList.h>
#include <arm_planning/OpenCloseGripper.h>
#include <arm_planning/ExtractAction.h>

void addCollisionObject ();

Executor::Ptr executor;
std::shared_ptr<Planning> planning;

void addCollisionObject ()
{
	moveit_msgs::CollisionObject collision_object;
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	collision_object.header.frame_id = "GETjag/base_link";
	collision_object.id = "box1";
	shape_msgs::SolidPrimitive primitive;
	primitive.type = primitive.BOX;
	primitive.dimensions.resize(3);
	primitive.dimensions[0] = 0.15;
	primitive.dimensions[1] = 0.001;
	primitive.dimensions[2] = 0.50;

	geometry_msgs::Pose box_pose;
	box_pose.orientation.w = 1.0;
	box_pose.position.x = -0.14;
	box_pose.position.y = 0.0;
	box_pose.position.z = 0.45;

	collision_object.primitives.push_back(primitive);
	collision_object.primitive_poses.push_back(box_pose);
	collision_object.operation = collision_object.ADD;
	std::vector<moveit_msgs::CollisionObject> collision_objects;
	collision_objects.push_back(collision_object);
	planning_scene_interface.applyCollisionObjects(collision_objects);

}

int main (int argc, char** argv)
{
	ros::init (argc, argv, "init");
	ros::NodeHandle nh;

	ros::AsyncSpinner spinner (0);
	spinner.start ();

	// This object is added to avoid the arm hitting with the pole that supports the Kinect camera.
	addCollisionObject ();

	auto paramList = std::make_shared<ArmControlParameterList> (nh);

	std::shared_ptr<JointStateSubscriber> jointStateSubscriber;
	jointStateSubscriber = std::make_shared<JointStateSubscriber> (nh, paramList);

	executor = std::make_shared<Executor> (nh, paramList, jointStateSubscriber);
	planning = std::make_shared<Planning> (nh, paramList, jointStateSubscriber);

	std::shared_ptr<GoHomeAction> goHome;
	goHome = std::make_shared<GoHomeAction> (nh, paramList, executor, planning);

	PoseGoalAction poseGoal (nh, paramList, executor, planning);
	GrabAction grab (nh, paramList, executor, planning);
	InspectAction inspect (nh, paramList, executor, planning);
	OpenCloseGripperAction openCloseGripper (nh, paramList, executor);
	ExtractAction extractAction (nh, paramList, executor, planning);
	DropObjectAction dropAction (nh, paramList, planning, executor);
	JoyControl control (nh, paramList, executor, planning, jointStateSubscriber);

	ros::waitForShutdown ();

	return 0;
}

