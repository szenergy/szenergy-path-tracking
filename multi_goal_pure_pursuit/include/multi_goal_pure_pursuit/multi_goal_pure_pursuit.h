/*
 * multi_Goal_pure_pursuit.h
 *
 *  Created on: Oct 19, 2019
 *      Author: kyberszittya
 */

#ifndef SRC_MULTI_GOAL_PURE_PURSUIT_H_
#define SRC_MULTI_GOAL_PURE_PURSUIT_H_

#include <multi_goal_pure_pursuit/trajectory_following.h>
#include <multi_goal_pure_pursuit/trajectory_lib.h>

#include <multi_goal_pure_pursuit/common.h>
#include <multi_goal_pure_pursuit/pure_pursuit_executor.h>
#include <multi_goal_pure_pursuit/trajectory_following_debug.h>

#include <thread>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>



class MultiGoalPurePursuit: public TrajectoryFollowing
{
private:
	// State
	std::shared_ptr<MultiGoalConfiguration> config;
	std::unique_ptr<AbstractPurePursuitExecutor> executor;
	// Debug
	std::unique_ptr<TrajectoryFollowingDebug> debug_ptr;
	// TF2
	tf2_ros::Buffer _buffer;
	geometry_msgs::TransformStamped _tr_base_link;
	geometry_msgs::TransformStamped _tr_wheelbase_link;
	tf2_ros::TransformListener _listener;
	// Control Thread
	ros::Timer control_timer;
	// ROS visualization
	ros::Publisher pub_next_waypoint_mark;
	ros::Publisher pub_circlecenter_mark;
	ros::Publisher pub_multi_goal_wpmarks;
	visualization_msgs::Marker next_waypoint_mark;
	visualization_msgs::Marker circle_center;
	visualization_msgs::MarkerArray multi_goal_waypoints;
public:

	MultiGoalPurePursuit(ros::NodeHandle& nh, std::shared_ptr<MultiGoalConfiguration> config):
		TrajectoryFollowing(nh),
		config(config),
		_buffer(ros::Duration(config->delay_pose*2)),
		_listener(_buffer)
	{
		switch (config->pp_type)
		{
		case PurePursuitType::PURE_PURSUIT: {
			executor = std::unique_ptr<SimplePurePursuitExecutor>(
					new SimplePurePursuitExecutor(state, config));
			break;
		}
		case PurePursuitType::MULTI_GOAL_PURE_PURSUIT: {
			executor = std::unique_ptr<MultiGoalPurePursuitExecutor>(
					new MultiGoalPurePursuitExecutor(state, config));
			break;
		}
		case PurePursuitType::CURVATURE_LOOKAHEAD: {
			executor = std::unique_ptr<CurvatureLookaheadChangingExecutor>(
					new CurvatureLookaheadChangingExecutor(state, config));
			break;
		}
		case PurePursuitType::LANE_CURVATURE_INTEGRAL: {
			executor = std::unique_ptr<LaneCurvatureIntegralExecutor>(
					new LaneCurvatureIntegralExecutor(state, config));
			break;
		}
		}
		// Debug intefaces
		if (config->debug)
		{
			debug_ptr = std::make_unique<TrajectoryFollowingDebug>();
			debug_ptr->init(nh, config->pp_type);
		}
	}

	void executeControlUpdate(geometry_msgs::TransformStamped _used_base_link);

	void control_timer_callback(const ros::TimerEvent&);

	virtual bool init();

	void velocityCommandUpdate();

	void directAngleControlCommand(const double& wheelangle,
			const double& linear_velocity,
			autoware_msgs::ControlCommandStamped* control_command);

	void visualizeCircleCenter(double angle);

	void visualizeMultiSelectedGoals(unsigned int offset,
			unsigned int multi_angle_goal_length);

};



#endif /* SRC_MULTI_GOAL_PURE_PURSUIT_H_ */
