#ifndef MULTIGOALSTATE_H
#define MULTIGOALSTATE_H

#include <Eigen/Dense>

#include <memory>

#include <eigen_conversions/eigen_msg.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/transform_datatypes.h>

enum SyncState {START, INITIALIZED};

struct TrajectoryFollowingState
{
	int current_lane_id;
	int closestwaypoint_id;
	int ld_id;
	Eigen::Affine3d pose;
	Eigen::Affine3d velocity;
	// Lateral deviation
	double lateral_deviation;
	// Lane received
	autoware_msgs::Lane lane;
	// Lane received
	autoware_msgs::Lane lane_base_waypoints;
	// Null-order state
	geometry_msgs::PoseStamped msg_pose;
	// First order state
	geometry_msgs::TwistStamped msg_velocity;
	// Control command
	autoware_msgs::ControlCommandStamped msg_command;
	// LookaheadDistance
	double lookahead_distance;
	// Visualization
	ros::Publisher pub_next_waypoint_mark;
	// Closest index from lookahead range
	int closest_ld_distance;
};

enum PurePursuitType{ PURE_PURSUIT, MULTI_GOAL_PURE_PURSUIT, CURVATURE_LOOKAHEAD, LANE_CURVATURE_INTEGRAL };

struct MultiGoalConfiguration
{
	const double delay_pose;
	const double min_look_ahead_distance;
	const double lookahead_ratio;
	const double wheelbase;
	const double update_hz;
	const double angle_limit;
	const unsigned int multi_goal_length;
	const double integral_lookahead;
	PurePursuitType pp_type;
	bool debug;
};



#endif
