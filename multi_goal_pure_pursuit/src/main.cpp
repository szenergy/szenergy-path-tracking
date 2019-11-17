#include <multi_goal_pure_pursuit/multi_goal_pure_pursuit.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "multi_goal_pure_pursuit");
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");
	// Private parameter
	int _type = 0;
	if (!private_nh.getParam("pp_type", _type))
	{
		ROS_WARN("No type specified, using original pure-pursuit");
	}
	double _freq = 40.0;
	if (!private_nh.getParam("freq", _freq))
	{
		ROS_WARN("No freq specified, using 40 Hz command update");
	}
	bool _debug = false;
	if (private_nh.getParam("debug", _debug))
	{
		ROS_INFO("Debug mode set");
	}
	double _angle_limit = 32.0;
	if (!private_nh.getParam("angle_limit", _angle_limit))
	{
		ROS_WARN("No angle limit set, using 32 deg");
	}
	std::shared_ptr<MultiGoalConfiguration> config;
	switch (_type)
	{
	case 0:
	{
		ROS_INFO("Using speed-ratio lookahead pure-pursuit");
		config = std::shared_ptr<MultiGoalConfiguration>(
				new MultiGoalConfiguration(
						{0.2, 3.0, 3.0,  2.7, _freq, _angle_limit,
							3, 2.1,
							PurePursuitType::PURE_PURSUIT,
							_debug
						})
		);
		break;
	}
	case 1:
	{
		ROS_INFO("Using multi-goal pure-pursuit");
		config = std::shared_ptr<MultiGoalConfiguration>(
				new MultiGoalConfiguration(
						{0.2, 3.0, 3.0,  2.7, _freq,
							_angle_limit,
							2, 2.1,
							PurePursuitType::MULTI_GOAL_PURE_PURSUIT,
							_debug
						})
		);
		break;
	}
	case 2:
	{
		ROS_INFO("Using curvature-lookahead pure-pursuit");
		config = std::shared_ptr<MultiGoalConfiguration>(
				new MultiGoalConfiguration(
						{0.2, 22.0, 3.0,  2.7, _freq,
							_angle_limit,
							3, 2.1,
							PurePursuitType::CURVATURE_LOOKAHEAD,
							_debug
						})
		);
		break;
	}
	case 3:
	{
		ROS_INFO("Using lane-curvature calculation");
		config = std::shared_ptr<MultiGoalConfiguration>(
				new MultiGoalConfiguration(
						{0.2, 3.0, 3.0,  2.7, _freq,
							_angle_limit,
							3, 2.1,
							PurePursuitType::LANE_CURVATURE_INTEGRAL,
							_debug
						})
		);
		break;
	}
	}

	// Multi-goal instantiation
	MultiGoalPurePursuit trajectory_following(nh, config);
	if (trajectory_following.init())
	{
		ros::spin();
		return 0;

	}
	else
	{
		return -1;
	}

}
