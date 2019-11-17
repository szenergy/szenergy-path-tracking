/*
 * trajectory_lib.h
 *
 *  Created on: Oct 19, 2019
 *      Author: kyberszittya
 */

#ifndef SRC_TRAJECTORY_LIB_H_
#define SRC_TRAJECTORY_LIB_H_

#include <Eigen/Dense>

#include <multi_goal_pure_pursuit/common.h>

#include <geometry_msgs/PoseStamped.h>
#include <autoware_msgs/ControlCommandStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <autoware_msgs/LaneArray.h>

constexpr double EPSILON_LINEAR_VELOCITY_SQR = 1e-3;

double distanceBetweenPoses(const geometry_msgs::Pose& pose0, const geometry_msgs::Pose& pose1);

double lengthLinearVelocity(const geometry_msgs::Twist& msg);

double calcCurvature(const geometry_msgs::Point& source,
		const geometry_msgs::Point& target,
		const geometry_msgs::Point& target_relative_frame);

double get2DPlaneDistance(const geometry_msgs::Point& source,
		const geometry_msgs::Point& target);

void curvatureToControlcommand(const geometry_msgs::TwistStamped current_velocity,
		const double curvature, const double target_velocity,
		const double wheelbase,
		const double dt,
		autoware_msgs::ControlCommandStamped* control_command);

double distanceBetweenPoints(const geometry_msgs::Point& p0, const geometry_msgs::Point& p1);

inline double curvatureToWheelAngle(const double curvature, const double& wheelbase)
{
	return atan(curvature * wheelbase);
}

double lowPassFilterRecursive(const double x, const double xm1, const double dt, const double alpha);

geometry_msgs::Point calcRadiusCenter(const double& yaw, const geometry_msgs::Point state, double radius);

double linearInterpolationDistance2D(const geometry_msgs::Point& p0,
		const geometry_msgs::Point& p1,
		const geometry_msgs::Point& state);

double integrateLaneLateralError(const geometry_msgs::TransformStamped& _tf_base_link,
		const autoware_msgs::Lane& lane, const int& t0, const int& tn);

double integrateLaneLateralErrorOrientation(
		const autoware_msgs::Lane& lane, const int& t0, const int& tn);

double linearInterpolationDistance2D(const geometry_msgs::Point& p0,
		const geometry_msgs::Point& p1,
		const geometry_msgs::Point& state);

double linearSliceLaneLateralIntegralError(const autoware_msgs::Lane& lane,
		const int& t0, const int& tn);

#endif /* SRC_TRAJECTORY_LIB_H_ */
