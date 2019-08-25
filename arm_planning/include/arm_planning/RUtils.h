/*
 * RUtils.h
 *
 *  Created on: Mar 2, 2019
 *      Author: mohammed
 */

#ifndef ROS_PG_WS_2018_19_ARM_PLANNING_INCLUDE_ARM_PLANNING_RUTILS_H_
#define ROS_PG_WS_2018_19_ARM_PLANNING_INCLUDE_ARM_PLANNING_RUTILS_H_


#include <ros/ros.h>


template<typename T>
void getParam (ros::NodeHandle& nh, const std::string& key, T& value, bool debug = false)
{
	if (not nh.getParam (key, value))
	{
		ROS_ERROR("parameter %s not set", key.c_str ());
	}
	else if (debug)
	{
		ROS_INFO("getParam (): %s ", key.c_str ());
	}
}



#endif /* ROS_PG_WS_2018_19_ARM_PLANNING_INCLUDE_ARM_PLANNING_RUTILS_H_ */
