#include <multi_goal_pure_pursuit/trajectory_lib.h>

double distanceBetweenPoses(const geometry_msgs::Pose& pose0, const geometry_msgs::Pose& pose1)
{
	double dx = pose0.position.x - pose1.position.x;
	double dy = pose0.position.y - pose1.position.y;
	double dz = pose0.position.z - pose1.position.z;
	return sqrt(dx*dx + dy*dy + dz*dz);
}

double lowPassFilterRecursive(const double x, const double xm1, const double dt, const double alpha)
{
	return alpha*(x) + (1.0-alpha)*xm1;
}


double distanceBetweenPoints(const geometry_msgs::Point& p0, const geometry_msgs::Point& p1)
{
	double dx = p0.x - p1.x;
	double dy = p0.y - p1.y;
	double dz = p0.z - p1.z;
	return sqrt(dx*dx + dy*dy + dz*dz);
}

double lengthLinearVelocity(const geometry_msgs::Twist& msg)
{
	double dx = msg.linear.x;
	double dy = msg.linear.y;
	double dz = msg.linear.z;
	return sqrt(dx*dx+dy*dy+dz*dz);
}

double get2DPlaneDistance(const geometry_msgs::Point& source, const geometry_msgs::Point& target)
{
	double dx = source.x - target.x;
	double dy = source.y - target.y;
	return sqrt(dx*dx + dy*dy);
}

double calcCurvature(const geometry_msgs::Point& source,
		const geometry_msgs::Point& target,
		const geometry_msgs::Point& target_relative_frame)
{
	double planar_distance = get2DPlaneDistance(source, target);
	return (2.0*target_relative_frame.y)/(planar_distance*planar_distance);
}

double linearSliceLaneLateralIntegralError(const autoware_msgs::Lane& lane,
		const int& t0, const int& tn)
{
	double integral_lateral_sum = 0.0;
	for (int i = t0; i < tn; i++)
	{
		integral_lateral_sum += linearInterpolationDistance2D(
				lane.waypoints[t0].pose.pose.position,
				lane.waypoints[tn].pose.pose.position,
				lane.waypoints[i].pose.pose.position
		);
	}
	return integral_lateral_sum;
}

double integrateLaneLateralError(const geometry_msgs::TransformStamped& _tf_base_link,
		const autoware_msgs::Lane& lane, const int& t0, const int& tn)
{
	geometry_msgs::PoseStamped waypoint_transformed;
	double integral_lateral_sum = 0.0;
	tf2::doTransform(lane.waypoints[t0].pose, waypoint_transformed, _tf_base_link);
	double ref_y = waypoint_transformed.pose.position.y;
	for (int i = t0; i < tn; i++)
	{
		tf2::doTransform(lane.waypoints[i].pose, waypoint_transformed, _tf_base_link);
		integral_lateral_sum += waypoint_transformed.pose.position.y - ref_y;
	}
	return integral_lateral_sum;
}

double integrateLaneLateralErrorOrientation(
		const autoware_msgs::Lane& lane, const int& t0, const int& tn)
{
	double integral_lateral_sum = 0.0;
	for (int i = t0; i < tn; i++)
	{

		integral_lateral_sum += std::tan(
				lane.waypoints[i].pose.pose.position.y - lane.waypoints[i - 1].pose.pose.position.y/
				lane.waypoints[i].pose.pose.position.x - lane.waypoints[i - 1].pose.pose.position.x
				);

	}
	return integral_lateral_sum;
}

geometry_msgs::Point calcRadiusCenter(const double& yaw, const geometry_msgs::Point state, double radius)
{
	Eigen::Matrix3d rot;
	rot << cos(yaw), -sin(yaw), 0.0,
		sin(yaw), cos(yaw), 0.0,
		0.0, 0.0, 1.0;
	geometry_msgs::Point p;
	Eigen::Vector3d v;
	v << 0.0, -radius, 0.0;
	v = rot * v;
	p.x = state.x + v.x();
	p.y = state.y + v.y();
	p.z = state.z + v.z();
	return p;
}


void curvatureToControlcommand(const geometry_msgs::TwistStamped current_velocity,
		const double curvature, const double target_velocity, const double wheelbase,
		const double dt,
		autoware_msgs::ControlCommandStamped* control_command)
{
	control_command->header.stamp = ros::Time::now();
	const double scalar_current_velocity = lengthLinearVelocity(current_velocity.twist);
	control_command->cmd.linear_velocity = target_velocity;
	if (scalar_current_velocity*scalar_current_velocity > EPSILON_LINEAR_VELOCITY_SQR)
	{
		// Filter stuff
		control_command->cmd.steering_angle = lowPassFilterRecursive(
			curvatureToWheelAngle(curvature, wheelbase),
			control_command->cmd.steering_angle, dt,
			0.78
		);
	}
	else
	{
		control_command->cmd.steering_angle = 0.0;
	}

}


double linearInterpolationDistance2D(const geometry_msgs::Point& p0,
		const geometry_msgs::Point& p1,
		const geometry_msgs::Point& state)
{
	Eigen::Vector3d _p0;
	tf::pointMsgToEigen(p0, _p0);
	_p0[2] = 0.0;
	Eigen::Vector3d _p1;
	tf::pointMsgToEigen(p1, _p1);
	_p1[2] = 0.0;
	Eigen::Vector3d _state;
	tf::pointMsgToEigen(state, _state);
	_state[2] = 0.0;
	auto v = _p1 - _p0;

	//std::cout << v << std::endl;
	return (v.cross(_state - _p1)).norm()/v.norm();
}
