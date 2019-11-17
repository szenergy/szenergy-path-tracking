/*
 * trajectory_following_debug.h
 *
 *  Created on: Nov 10, 2019
 *      Author: kyberszittya
 */

#ifndef INCLUDE_MULTI_GOAL_PURE_PURSUIT_TRAJECTORY_FOLLOWING_DEBUG_H_
#define INCLUDE_MULTI_GOAL_PURE_PURSUIT_TRAJECTORY_FOLLOWING_DEBUG_H_

#include <multi_goal_pure_pursuit/common.h>

class TrajectoryFollowingDebug
{
private:
	// Publish pure pursuit type
	ros::Publisher pub_pure_pursuit_type;
	std_msgs::Int32 msg_pure_pursuit_type;
	// Lookahead publisher
	ros::Publisher pub_lookahead;
	std_msgs::Float64 msg_lookahead_distance;
	ros::Publisher pub_lateral_deviation;
	std_msgs::Float64 msg_lateral_deviation;
	// Publish integral error
	ros::Publisher pub_integral_lateral_error;
	std_msgs::Float64 msg_integral_lateral_error;
	// Debug waypoints
	visualization_msgs::Marker msg_closest_waypoint;
	ros::Publisher pub_closest_waypoint;
	visualization_msgs::Marker msg_closest_waypoint_tp1;
	ros::Publisher pub_closest_waypoint_tp1;
	// Lateral error on distance
	// Debug pose
	visualization_msgs::Marker msg_current_pose_marker;
	ros::Publisher pub_current_pose_marker;
	// Publish time of calculation
	ros::Publisher pub_time_of_calulation;
	std_msgs::Float64 msg_calc_time;
public:
	void init(ros::NodeHandle& nh, PurePursuitType pp_type)
	{
		// Visualize waypoints
		msg_closest_waypoint.color.r = 1.0;
		msg_closest_waypoint.color.a = 1.0;
		msg_closest_waypoint.scale.x = 1.0;
		msg_closest_waypoint.scale.y = 1.0;
		msg_closest_waypoint.scale.z = 1.0;
		msg_closest_waypoint.type = visualization_msgs::Marker::SPHERE;
		msg_closest_waypoint.header.frame_id = "map";
		msg_closest_waypoint_tp1.color.r = 1.0;
		msg_closest_waypoint_tp1.color.a = 1.0;
		msg_closest_waypoint_tp1.scale.x = 1.0;
		msg_closest_waypoint_tp1.scale.y = 1.0;
		msg_closest_waypoint_tp1.scale.z = 1.0;
		msg_closest_waypoint_tp1.type = visualization_msgs::Marker::SPHERE;
		msg_closest_waypoint_tp1.header.frame_id = "map";
		// Current pose marker
		msg_current_pose_marker.header.frame_id = "map";
		msg_current_pose_marker.type = visualization_msgs::Marker::SPHERE;
		msg_current_pose_marker.color.r = 1.0;
		msg_current_pose_marker.color.a = 1.0;
		msg_current_pose_marker.scale.x = 1.0;
		msg_current_pose_marker.scale.y = 1.0;
		msg_current_pose_marker.scale.z = 1.0;
		//
		pub_pure_pursuit_type = nh.advertise<std_msgs::Int32>("trajectory_following/debug/pp_type", 10, true);
		msg_pure_pursuit_type.data = pp_type;
		pub_pure_pursuit_type.publish(msg_pure_pursuit_type);
		// Lookahead debug information
		pub_lookahead = nh.advertise<std_msgs::Float64>("trajectory_following/debug/lookahead_distance", 10);
		pub_lateral_deviation = nh.advertise<std_msgs::Float64>("trajectory_following/debug/lateral_deviation", 10);
		// Waypoints
		pub_closest_waypoint = nh.advertise<visualization_msgs::Marker>("trajectory_following/debug/closest_waypoint", 10);
		pub_closest_waypoint_tp1 = nh.advertise<visualization_msgs::Marker>("trajectory_following/debug/closest_waypoint_tp1", 10);
		// Current pose marker
		pub_current_pose_marker = nh.advertise<visualization_msgs::Marker>("trajectory_following/debug/current_pose_marker", 10);
		// Integral lateral error
		pub_integral_lateral_error = nh.advertise<std_msgs::Float64>("trajectory_following/debug/integral_lateral_error", 10);
		// Time of calculation
		pub_time_of_calulation = nh.advertise<std_msgs::Float64>("trajectory_following/debug/time_of_calculation", 10);
	}

	void publishLookaheadDistance(const double &ld)
	{
		msg_lookahead_distance.data = ld;
		pub_lookahead.publish(msg_lookahead_distance);
	}

	void publishLateralDeviation(const double &lateral_deviation)
	{
		msg_lateral_deviation.data = lateral_deviation;
		pub_lateral_deviation.publish(msg_lateral_deviation);
	}

	void publishClosestWaypoints(const geometry_msgs::Point& wp0, const geometry_msgs::Point& wp1)
	{
		msg_closest_waypoint.header.stamp = ros::Time::now();
		msg_closest_waypoint.pose.position = wp0;
		msg_closest_waypoint_tp1.pose.position = wp1;
		msg_closest_waypoint_tp1.header.stamp = ros::Time::now();
		pub_closest_waypoint.publish(msg_closest_waypoint);
		pub_closest_waypoint_tp1.publish(msg_closest_waypoint_tp1);

	}

	void publishCurrentPose(const geometry_msgs::Pose& pose)
	{
		msg_current_pose_marker.pose = pose;
		msg_current_pose_marker.header.stamp = ros::Time::now();
		pub_current_pose_marker.publish(msg_current_pose_marker);
	}

	void publishLateralIntegralError(const double& lateral_error)
	{
		msg_integral_lateral_error.data = lateral_error;
		pub_integral_lateral_error.publish(msg_integral_lateral_error);
	}

	void publishTimeOfCalculation(const double& time_of_calc)
	{
		msg_calc_time.data = time_of_calc;
		pub_time_of_calulation.publish(msg_calc_time);
	}
};


#endif /* INCLUDE_MULTI_GOAL_PURE_PURSUIT_TRAJECTORY_FOLLOWING_DEBUG_H_ */
