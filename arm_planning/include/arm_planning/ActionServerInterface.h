/*
 * ActionServerInterface.h
 *
 *  Created on: Feb 6, 2019
 *      Author: mohammed
 */

#ifndef ARM_PLANNING_INCLUDE_ACTIONSERVERINTERFACE_H_
#define ARM_PLANNING_INCLUDE_ACTIONSERVERINTERFACE_H_

#include <geometry_msgs/Pose.h>
#include <yaml-cpp/yaml.h>
#include <memory>

class ActionServerInterface
{
	public:
		virtual ~ActionServerInterface () = default;
		virtual void setFeedback (double f) = 0;
		virtual void setSucceeded () = 0;
		virtual void setPreempted () = 0;
		virtual bool isActive() = 0;
		using ActionServerInterfacePtr = std::shared_ptr<ActionServerInterface>;

	private:
//		virtual void convertGoapMsgToPose(const std::string& goapMsg, geometry_msgs::Pose& pose);
};



#endif /* ARM_PLANNING_INCLUDE_ACTIONSERVERINTERFACE_H_ */
