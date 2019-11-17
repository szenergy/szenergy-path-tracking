/*
 * pure_pursuit_control.cpp
 *
 *  Created on: Nov 7, 2019
 *      Author: kyberszittya
 */


#include <multi_goal_pure_pursuit/multi_goal_pure_pursuit.h>
#include <multi_goal_pure_pursuit/pure_pursuit_executor.h>

void MultiGoalPurePursuit::executeControlUpdate(geometry_msgs::TransformStamped _used_base_link)
{
	double dist = 0.0;
	geometry_msgs::Pose prev_pose;
	// We intend to calc time
	auto start_time = ros::Time::now();
	//
	if (state->lane.waypoints.size() > 0)
	{
		prev_pose = state->lane.waypoints[0].pose.pose;
	}
	else
	{
		prev_pose = state->msg_pose.pose;
	}
	double ld = executor->calcLookaheadDistance();
	ld = lowPassFilterRecursive(ld, state->closest_ld_distance, 1.0/config->update_hz, 0.75);

	if (config->debug)
	{
		debug_ptr->publishLookaheadDistance(ld);
		if (state->lane.waypoints.size()>=1
				&& state->closestwaypoint_id != -1
				&& state->lane_base_waypoints.waypoints.size() >= 1)
		{

			double distance = linearInterpolationDistance2D(
					state->lane_base_waypoints.waypoints[state->closestwaypoint_id].pose.pose.position,
					state->lane_base_waypoints.waypoints[state->closestwaypoint_id+1].pose.pose.position,
					state->msg_pose.pose.position);
			//std::cout << state->lane_base_waypoints.waypoints[state->closestwaypoint_id].pose.pose.position << std::endl;
			//std::cout << state->lane_base_waypoints.waypoints[state->closestwaypoint_id+1].pose.pose.position << std::endl;
			//std::cout << state->msg_pose.pose.position << std::endl;
			if (distance>0.0)
			{
				debug_ptr->publishLateralDeviation(distance);
				debug_ptr->publishClosestWaypoints(
					state->lane_base_waypoints.waypoints[state->closestwaypoint_id].pose.pose.position,
					state->lane_base_waypoints.waypoints[state->closestwaypoint_id+1].pose.pose.position
				);
				debug_ptr->publishCurrentPose(state->msg_pose.pose);
			}
		}
	}

	for (int i = 0; i < state->lane.waypoints.size(); i++)
	{
		geometry_msgs::PoseStamped waypoint_transformed;

		if (dist > ld)
		{
			tf2::doTransform(state->lane.waypoints[i].pose, waypoint_transformed, _tr_base_link);
			double target_velocity = lengthLinearVelocity(state->lane.waypoints[i].twist.twist);
			executor->updateControlCommand(i, waypoint_transformed, target_velocity);
			// Visualization
			switch(config->pp_type)
			{
			case MULTI_GOAL_PURE_PURSUIT:
			{
				visualizeMultiSelectedGoals(i, config->multi_goal_length);
				break;
			}
			}
			state->closest_ld_distance = i;
			next_waypoint_mark.pose.position.x = waypoint_transformed.pose.position.x;
			next_waypoint_mark.pose.position.y = waypoint_transformed.pose.position.y;
			next_waypoint_mark.pose.position.z = waypoint_transformed.pose.position.z;
			next_waypoint_mark.header.stamp = ros::Time::now();
			pub_next_waypoint_mark.publish(next_waypoint_mark);
			state->ld_id = i;
			break;
		}
		dist += distanceBetweenPoses(prev_pose, state->lane.waypoints[i].pose.pose);
		prev_pose = state->lane.waypoints[i].pose.pose;
	}
	state->lookahead_distance = ld;
	// Lets watch stuff
	auto end_time = ros::Time::now();
	if (config->debug)
	{
		debug_ptr->publishTimeOfCalculation(end_time.toSec() - start_time.toSec());
	}
}

void SimplePurePursuitExecutor::updateControlCommand(const int i,
		const geometry_msgs::PoseStamped& waypoint_transformed,
		const double& target_velocity)
{
	double curvature = calcCurvature(
			state->msg_pose.pose.position,
			state->lane.waypoints[i].pose.pose.position,
			waypoint_transformed.pose.position
	);
	curvatureToControlcommand(state->msg_velocity, curvature, target_velocity,
			config->wheelbase,
			config->update_hz,
			&state->msg_command);
}


void MultiGoalPurePursuitExecutor::updateControlCommand(const int i,
		const geometry_msgs::PoseStamped& waypoint_transformed,
		const double& target_velocity)
{
	double best_angle = multiAngleCurvature(i, config->multi_goal_length);

	double steer_angle = atan(2*config->wheelbase*sin(best_angle));
	curvatureToControlcommand(state->msg_velocity, best_angle, target_velocity,
			config->wheelbase, 1.0/config->update_hz, &state->msg_command);
}

double MultiGoalPurePursuitExecutor::multiAngleCurvature(const unsigned int offset,
		const unsigned int multi_angle_goal_length)
{
	double ang = -config->angle_limit*M_PI/180.0;
	double d_ang = 0.1*M_PI/180.0;
	double ang_max = config->angle_limit*M_PI/180.0;

	unsigned int i = 0;
	unsigned int min_i = 0;
	std::vector<double> angles;
	std::vector<double> dsums;
	tf::Quaternion quat;
	double yaw = tf::getYaw(state->msg_pose.pose.orientation);

	while(true)
	{
		if (ang >= ang_max){
			break;
		}
		double d_sum = 0;
		double rho_i = -2.0/ang;
		const geometry_msgs::Point center = calcRadiusCenter(yaw, state->msg_pose.pose.position, rho_i);
		int shifted_offset = std::max<int>(0, offset - multi_angle_goal_length);
		for (int j = 0;	j < multi_angle_goal_length*2; j++)
		{
			double d = distanceBetweenPoints(
					state->lane.waypoints[shifted_offset + j].pose.pose.position, center);
			d_sum += abs(d	- abs(rho_i));
		}
		angles.push_back(ang);
		dsums.push_back(d_sum);
		if (d_sum < dsums[min_i])
		{
			min_i = i;
		}
		ang += d_ang;
		i++;
	}
	return angles[min_i];
}


void LaneCurvatureIntegralExecutor::updateControlCommand(const int i,
		const geometry_msgs::PoseStamped& waypoint_transformed,
		const double& target_velocity)
{
	// We check things based on lateral error
	double lat_error = 0.0;
	unsigned int selected_goals = 1;
	// If there are any
	if (state->lane.waypoints.size() > 0)
	{

		int integral_offset = std::min<int>(
				state->ld_id, state->lane.waypoints.size());
		lat_error = linearSliceLaneLateralIntegralError(
				state->lane, 0, integral_offset);
		lat_error /= state->lookahead_distance;

	}
	if (lat_error < THRESHOLD_SPEED_RATIO && lat_error > -THRESHOLD_SPEED_RATIO)
	{
		// The thing is very simple: use speed-ratio lookahead

		double curvature = calcCurvature(
					state->msg_pose.pose.position,
					state->lane.waypoints[i].pose.pose.position,
					waypoint_transformed.pose.position
		);
		curvatureToControlcommand(state->msg_velocity, curvature, target_velocity,
				config->wheelbase,
				config->update_hz,
				&state->msg_command);
	}
	else
	{
		if (std::abs(lat_error) >= THRESHOLD_SPEED_RATIO && std::abs(lat_error) < THRESHOLD_TWO_GOAL)
		{
			selected_goals = 2;
		}
		else if (std::abs(lat_error) >= THRESHOLD_TWO_GOAL)
		{
			selected_goals = 3;


		}
		double best_angle = multiAngleCurvature(i, selected_goals);

		double steer_angle = atan(2*config->wheelbase*sin(best_angle));
		curvatureToControlcommand(state->msg_velocity, best_angle, target_velocity,
						config->wheelbase, 1.0/config->update_hz, &state->msg_command);

	}
	std::cout << lat_error << '\t' << selected_goals << '\t' <<
			get2DPlaneDistance(state->msg_pose.pose.position, state->lane.waypoints[0].pose.pose.position) << std::endl;

}
