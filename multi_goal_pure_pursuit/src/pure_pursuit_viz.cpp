/*
 * pure_pursuit_viz.cpp
 *
 *  Created on: Nov 7, 2019
 *      Author: kyberszittya
 */


#include <multi_goal_pure_pursuit/multi_goal_pure_pursuit.h>

void MultiGoalPurePursuit::visualizeCircleCenter(double angle)
{
	double rho_i = -2.0/angle;
	double yaw = tf::getYaw(state->msg_pose.pose.orientation);
	const geometry_msgs::Point center = calcRadiusCenter(yaw, state->msg_pose.pose.position, rho_i);
	circle_center.pose.position.x = center.x - state->msg_pose.pose.position.x;
	circle_center.pose.position.y = center.y - state->msg_pose.pose.position.y;
	circle_center.pose.position.z = center.z - state->msg_pose.pose.position.z;
	pub_circlecenter_mark.publish(circle_center);
}

void MultiGoalPurePursuit::visualizeMultiSelectedGoals(unsigned int offset,
		unsigned int multi_angle_goal_length)
{
	this->multi_goal_waypoints.markers.clear();
	auto st = ros::Time::now();
	for (unsigned int j = state->current_lane_id + offset;
						j < state->current_lane_id + offset + multi_angle_goal_length; j++)
	{
		visualization_msgs::Marker m;
		m.header.stamp = st;
		m.header.frame_id = "map";
		m.pose.position.x = state->lane.waypoints[j].pose.pose.position.x;
		m.pose.position.y = state->lane.waypoints[j].pose.pose.position.y;
		m.pose.position.z = state->lane.waypoints[j].pose.pose.position.z;
		m.type = visualization_msgs::Marker::SPHERE;
		m.pose.orientation.x = 0.0;
		m.pose.orientation.y = 0.0;
		m.pose.orientation.z = 0.0;
		m.pose.orientation.w = 1.0;
		m.scale.x = 1.0;
		m.scale.y = 1.0;
		m.scale.z = 1.0;
		m.color.a = 1.0;
		m.color.r = 1.0;
		m.color.g = 0.0;
		m.color.b = 1.0;
		multi_goal_waypoints.markers.push_back(m);

	}
	pub_multi_goal_wpmarks.publish(multi_goal_waypoints);
}
