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

	static double calcLsRadius(unsigned int n, unsigned int offset, const geometry_msgs::Point start_point,
			const autoware_msgs::Lane& lane)
	{
		// See the following articles:
		// Circle fitting by linear and nonlinear least squares; I. D. Coope
		// Least-squares fitting of circles and ellipses; Walter GanderGene H. GolubRolf Strebel
		Eigen::MatrixXd points(n+1, 2);
		points(0,0) = start_point.x;
		points(0,1) = start_point.y;
		for (int i = 0; i < n; i++)
		{
			points(i+1,0) = lane.waypoints[offset+i].pose.pose.position.x;
			points(i+1,1) = lane.waypoints[offset+i].pose.pose.position.y;
		}
		Eigen::MatrixXd Q(points.rows(), points.cols() + 1);
		Q.block(0,0,n+1, points.cols()) = points;
		Q.col(2) = Eigen::VectorXd::Ones(points.rows(), 1);
		Eigen::VectorXd res = Q.colPivHouseholderQr().solve(Q.cwiseProduct(Q).rowwise().sum());
		Eigen::VectorXd x = 0.5*res.segment(0,2);
		return std::sqrt(res(Q.cols()-1) + x.dot(x.transpose()));
	}

	static double calcLsRadiusBackward(unsigned int n, unsigned int offset, const geometry_msgs::Point start_point,
				const autoware_msgs::Lane& lane)
	{
		// See the following articles:
		// Circle fitting by linear and nonlinear least squares; I. D. Coope
		// Least-squares fitting of circles and ellipses; Walter GanderGene H. GolubRolf Strebel
		Eigen::MatrixXd points(n*2+1, 2);
		points(0,0) = start_point.x;
		points(0,1) = start_point.y;
		for (int i = -n; i < n; i++)
		{
			points(i+1,0) = lane.waypoints[offset+i].pose.pose.position.x;
			points(i+1,1) = lane.waypoints[offset+i].pose.pose.position.y;
		}
		Eigen::MatrixXd Q(points.rows(), points.cols() + 1);
		Q.block(0,0,n+1, points.cols()) = points;
		Q.col(2) = Eigen::VectorXd::Ones(points.rows(), 1);
		Eigen::VectorXd res = Q.colPivHouseholderQr().solve(Q.cwiseProduct(Q).rowwise().sum());
		Eigen::VectorXd x = 0.5*res.segment(0,2);
		return std::sqrt(res(Q.cols()-1) + x.dot(x.transpose()));
	}

};



#endif /* SRC_MULTI_GOAL_PURE_PURSUIT_H_ */
