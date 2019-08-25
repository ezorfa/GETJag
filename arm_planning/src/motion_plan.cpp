#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <actionlib/client/simple_action_client.h>
#include <handling_msgs/PoseGoalAction.h>

class SubscribeAndPublish {
public:
	SubscribeAndPublish(ros::NodeHandle& nh_) :
			n_(nh_), ac("/dexterity/goalPose", true) {

	}
	void goal_call();

private:
	ros::NodeHandle n_;
	actionlib::SimpleActionClient<handling_msgs::PoseGoalAction> ac;
	handling_msgs::PoseGoalActionGoal goal;

};

void SubscribeAndPublish::goal_call() {

	geometry_msgs::Pose pose_man;
//
//	pose_man.orientation.x = 0.6679;
//	pose_man.orientation.y = -0.23221;
//	pose_man.orientation.z = -0.6678;
//	pose_man.orientation.w = -0.2322;
//
//	pose_man.position.x = 0.496919;
//	pose_man.position.y = -0.4;
//	pose_man.position.z = 0.231986;

//	pose_man.orientation.x = 0;
//	pose_man.orientation.y = 0;
//	pose_man.orientation.z = 0;
//	pose_man.orientation.w = 1;
//
//	pose_man.position.x = 0.5;
//	pose_man.position.y = -0.2;
//	pose_man.position.z = 0.6;
//
//
	pose_man.orientation.x = 0.6679;
	pose_man.orientation.y = -0.23221;
	pose_man.orientation.z = -0.6678;
	pose_man.orientation.w = -0.2322;

	pose_man.position.x = 0.496919;
	pose_man.position.y = 0.000331664;
	pose_man.position.z = 0.231986;

	ROS_INFO_STREAM(pose_man);
	ac.waitForServer();
	ROS_INFO_STREAM("WAITED");
	goal.goal.pose = pose_man;
	ac.sendGoal(goal.goal);
//	ROS_INFO("END EFFECTOR FRAME: %s", move_group.getEndEffectorLink().c_str());




}

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "move_group_interface");
	ros::NodeHandle nh;
	SubscribeAndPublish SAP(nh);
	SAP.goal_call();
	ros::spin();
}

