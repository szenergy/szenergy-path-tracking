/*
 * trajectory_following.cpp
 *
 *  Created on: Oct 19, 2019
 *      Author: kyberszittya
 */

#include <multi_goal_pure_pursuit/trajectory_lib.h>
#include <multi_goal_pure_pursuit/trajectory_following.h>

#include <iostream>



void TrajectoryFollowing::cbcurrent_pose(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
	switch(sync_state)
	{
	case INITIALIZED: {
		state->msg_pose = *msg;
		break;
	}
	default:{
		break;
	}
	}
}

void TrajectoryFollowing::cbfinal_waypoints(const autoware_msgs::Lane::ConstPtr &msg)
{
	state->lane = *msg;
	switch(sync_state)
	{
	case START:
	{
		ROS_INFO_STREAM("Initialized for final waypoints");
		sync_state = INITIALIZED;
		break;
	}
	case INITIALIZED:
	{
		break;
	}
	}
}

void TrajectoryFollowing::cbcurrent_velocity(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
	state->msg_velocity = *msg;
}

void TrajectoryFollowing::cbclosest_waypoints(const std_msgs::Int32::ConstPtr &msg)
{
	state->closestwaypoint_id = msg->data;
}

void TrajectoryFollowing::cbcurrent_lane_id(const std_msgs::Int32::ConstPtr &msg)
{
	state->current_lane_id = msg->data;
}

void TrajectoryFollowing::cbbase_waypoints(const autoware_msgs::Lane::ConstPtr &msg)
{
	state->lane_base_waypoints = *msg;
}
