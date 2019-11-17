/*
 * pure_pursuit_executor.h
 *
 *  Created on: Nov 7, 2019
 *      Author: kyberszittya
 */

#ifndef SRC_PURE_PURSUIT_EXECUTOR_H_
#define SRC_PURE_PURSUIT_EXECUTOR_H_

#include "trajectory_lib.h"
#include "multi_goal_state.h"


#include <tf_conversions/tf_eigen.h>

class AbstractPurePursuitExecutor
{
protected:
	std::shared_ptr<TrajectoryFollowingState> state;
	std::shared_ptr<MultiGoalConfiguration> config;
public:
	AbstractPurePursuitExecutor(std::shared_ptr<TrajectoryFollowingState> state,
			std::shared_ptr<MultiGoalConfiguration> config): state(state), config(config){}
	virtual ~AbstractPurePursuitExecutor() = default;

	virtual void updateControlCommand(const int i,
			const geometry_msgs::PoseStamped& waypoint_transformed,
			const double& target_velocity) = 0;

	virtual double calcLookaheadDistance()
	{
		return std::max(config->min_look_ahead_distance + config->wheelbase,
				lengthLinearVelocity(state->msg_velocity.twist) * config->lookahead_ratio
				+ config->wheelbase);
	}
};

class SimplePurePursuitExecutor: public AbstractPurePursuitExecutor
{
public:
	SimplePurePursuitExecutor(std::shared_ptr<TrajectoryFollowingState> state,
			std::shared_ptr<MultiGoalConfiguration> config):
				AbstractPurePursuitExecutor(state, config){}

	virtual void updateControlCommand(const int i,
			const geometry_msgs::PoseStamped& waypoint_transformed,
			const double& target_velocity);
};

class MultiGoalPurePursuitExecutor: public AbstractPurePursuitExecutor
{
public:
	MultiGoalPurePursuitExecutor(std::shared_ptr<TrajectoryFollowingState> state,
			std::shared_ptr<MultiGoalConfiguration> config):
				AbstractPurePursuitExecutor(state, config){}

	virtual void updateControlCommand(const int i,
			const geometry_msgs::PoseStamped& waypoint_transformed,
			const double& target_velocity);

	double multiAngleCurvature(const unsigned int offset,
				const unsigned int multi_angle_goal_length);

};

constexpr double THRESHOLD_SPEED_RATIO = 0.15;
constexpr double THRESHOLD_TWO_GOAL = 0.4;

class LaneCurvatureIntegralExecutor: public MultiGoalPurePursuitExecutor
{
public:
		LaneCurvatureIntegralExecutor(std::shared_ptr<TrajectoryFollowingState> state,
				std::shared_ptr<MultiGoalConfiguration> config):
					MultiGoalPurePursuitExecutor(state, config)
	    { }

		virtual void updateControlCommand(const int i,
					const geometry_msgs::PoseStamped& waypoint_transformed,
					const double& target_velocity);


};

class CurvatureLookaheadChangingExecutor: public SimplePurePursuitExecutor
{
protected:
	std::vector<double> relative_angles;
public:
	CurvatureLookaheadChangingExecutor(std::shared_ptr<TrajectoryFollowingState> state,
			std::shared_ptr<MultiGoalConfiguration> config):
				SimplePurePursuitExecutor(state, config)
    {
		relative_angles.reserve(6);
    }
	/*
	updateControlCommand(const int i,
					const geometry_msgs::PoseStamped& waypoint_transformed,
					const double& target_velocity)*/

	double calcAngleDisparity()
	{
		relative_angles.clear();
		// The current closest waypoint is 0
		tf::Transform _tf_pose;
		tf::Transform _tf_closest;
		tf::poseMsgToTF(state->lane.waypoints[0].pose.pose, _tf_closest);
		tf::poseMsgToTF(state->msg_pose.pose, _tf_pose);
		relative_angles.push_back(tf::getYaw(_tf_pose.inverseTimes(_tf_closest).getRotation()));
		// Get yaw to closest waypoint to lookahead distance
		tf::Transform _tf_ld;
		tf::poseMsgToTF(state->lane.waypoints[state->closest_ld_distance].pose.pose, _tf_ld);
		relative_angles.push_back(tf::getYaw(_tf_closest.inverseTimes(_tf_ld).getRotation()));
		// Get another set of waypoints (if there are any)
		// The index i is the next closest lookahead point
		for (unsigned int j = 0; j < 2; j++)
		{
			if (state->closest_ld_distance > 2){
				tf::Transform _tf_wp_jm;
				tf::poseMsgToTF(state->lane.waypoints[
					state->closest_ld_distance-j].pose.pose, _tf_wp_jm);
				relative_angles.push_back(
					tf::getYaw(_tf_closest.inverseTimes(_tf_wp_jm).getRotation()));
			}
			tf::Transform _tf_wp_jp;
			tf::poseMsgToTF(state->lane.waypoints[state->closest_ld_distance+j].pose.pose, _tf_wp_jp);
			relative_angles.push_back(tf::getYaw(_tf_closest.inverseTimes(_tf_wp_jp).getRotation()));
		}
		for (const auto& v: relative_angles)
		{
			std::cout << v << ' ';
		}
		std::cout << '\n';
		auto max_ang = std::max_element(relative_angles.begin(), relative_angles.end());
		auto min_ang = std::min_element(relative_angles.begin(), relative_angles.end());
		double d_ang = *max_ang - *min_ang;
	}


	virtual double calcLookaheadDistance()
	{
		// Calculate lookahead distance
		double dist = 0.0;
		geometry_msgs::Pose prev_pose = state->msg_pose.pose;
		double ld = AbstractPurePursuitExecutor::calcLookaheadDistance();
		for (int i = 0; i < state->lane.waypoints.size(); i++)
		{
			dist += distanceBetweenPoses(prev_pose, state->lane.waypoints[i].pose.pose);
			prev_pose = state->lane.waypoints[i].pose.pose;
			if (dist > ld)
			{
				state->closest_ld_distance = i;
				break;
			}
		}

		// Store angles



		// Get yaw to closest waypoint
		relative_angles.clear();
		if (state->lane.waypoints.size() > 0)
		{
			double d_ang = calcAngleDisparity();
			ld = d_ang * config->lookahead_ratio + config->wheelbase + config->min_look_ahead_distance;

			return ld;
		}
		else
		{
			return ld;
		}
	}

	/*
	void updateControlCommand(const int i,
			const geometry_msgs::PoseStamped& waypoint_transformed,
			const double& target_velocity)
	{
		//double best_angle = multiAngleCurvature(i, config->multi_goal_length);
		double best_angle = calcAngleDisparity();

		double steer_angle = atan(2*config->wheelbase*sin(best_angle));
		curvatureToControlcommand(state->msg_velocity, best_angle, target_velocity,
				config->wheelbase, &state->msg_command);
	}
	*/
};

#endif /* SRC_PURE_PURSUIT_EXECUTOR_H_ */
