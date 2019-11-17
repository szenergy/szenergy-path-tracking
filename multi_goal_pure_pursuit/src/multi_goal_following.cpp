/*
 * multi_goal_following.cpp
 *
 *  Created on: Oct 19, 2019
 *      Author: kyberszittya
 */

#include <multi_goal_pure_pursuit/multi_goal_pure_pursuit.h>
#include <multi_goal_pure_pursuit/trajectory_lib.h>

#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <eigen_conversions/eigen_msg.h>



bool MultiGoalPurePursuit::init()
{
	if ( TrajectoryFollowing::init()){
		// TODO: initialization might screw up on TF correct it
		// TODO: ROS timer is quite elegant, use it
		// Control command setup
		state->msg_command.header.frame_id = "base_link";
		// Additional publishers
		pub_next_waypoint_mark = nh.advertise<visualization_msgs::Marker>("next_waypoint_mark", 10);
		pub_circlecenter_mark = nh.advertise<visualization_msgs::Marker>("multigoal/circle_center", 10);
		pub_multi_goal_wpmarks = nh.advertise<visualization_msgs::MarkerArray>("multigoal/selected_wpmarks", 10);
		control_timer = nh.createTimer(
			ros::Duration(1.0/config->update_hz),
			&MultiGoalPurePursuit::control_timer_callback, this
		);
		// Visualization
		next_waypoint_mark.scale.x = 1.0;
		next_waypoint_mark.scale.y = 1.0;
		next_waypoint_mark.scale.z = 1.0;
		next_waypoint_mark.pose.orientation.w = 1.0;
		next_waypoint_mark.color.r = 1.0;
		next_waypoint_mark.color.g = 1.0;
		next_waypoint_mark.color.b = 0.0;
		next_waypoint_mark.color.a = 1.0;
		next_waypoint_mark.header.frame_id = "base_link";
		next_waypoint_mark.text = "next_waypoint";
		next_waypoint_mark.type = visualization_msgs::Marker::SPHERE;
		// Init circle center mark
		circle_center.scale.x = 1.0;
		circle_center.scale.y = 1.0;
		circle_center.scale.z = 1.0;
		circle_center.pose.orientation.w = 1.0;
		circle_center.color.r = 0.0;
		circle_center.color.g = 1.0;
		circle_center.color.b = 0.0;
		circle_center.color.a = 1.0;
		circle_center.header.frame_id = "base_link";
		circle_center.text = "center_circle";
		circle_center.type = visualization_msgs::Marker::SPHERE;
		multi_goal_waypoints.markers.reserve(config->multi_goal_length);
		return true;
	}
	return false;
}

void MultiGoalPurePursuit::control_timer_callback(const ros::TimerEvent&)
{
	velocityCommandUpdate();
	state->msg_command.header.stamp = ros::Time::now();
	this->pub_ctrl_cmd.publish(state->msg_command);
}

void MultiGoalPurePursuit::directAngleControlCommand(const double& wheelangle,
		const double& linear_velocity,
		autoware_msgs::ControlCommandStamped* control_command)
{
	control_command->cmd.steering_angle = wheelangle;
	control_command->cmd.linear_velocity = linear_velocity;
}

void MultiGoalPurePursuit::velocityCommandUpdate()
{

	try
		{
		geometry_msgs::TransformStamped _base_link_update = _buffer.lookupTransform(
			"base_link",
			"map", state->msg_pose.header.stamp, ros::Duration(config->delay_pose));
		_tr_base_link = _base_link_update;

	}
	catch(tf2::LookupException& e)
	{
		ROS_ERROR_STREAM("Unable to find transform " << e.what());
	}
	catch(tf2::ExtrapolationException& e)
	{
		ROS_ERROR_STREAM("Delay in TF, using latest transformation: " << e.what());
	}
	catch(tf2::ConnectivityException& e)
	{
		ROS_ERROR_STREAM("Connectivity exception: " << e.what());
	}
	executeControlUpdate(_tr_base_link);

}
