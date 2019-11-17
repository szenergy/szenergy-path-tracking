#include <multi_goal_pure_pursuit/trajectory_following.h>


bool TrajectoryFollowing::init()
{
	/// Initialize publishers
	pub_ctrl_cmd = nh.advertise<autoware_msgs::ControlCommandStamped>("ctrl_cmd", 10);
    /// Initialize subscribers
    /// Callback Int32 initialization
    sub_current_lane_id = nh.subscribe("current_lane_id", 10, &TrajectoryFollowing::cbcurrent_lane_id, this);
    /// Callback Int32 initialization
    sub_closest_waypoints = nh.subscribe("closest_waypoint", 10, &TrajectoryFollowing::cbclosest_waypoints, this);
    /// Callback PoseStamped initialization
    sub_current_pose = nh.subscribe("current_pose", 10, &TrajectoryFollowing::cbcurrent_pose, this);
    /// Callback LaneArray initialization
    sub_final_waypoints = nh.subscribe("final_waypoints", 10, &TrajectoryFollowing::cbfinal_waypoints, this);
    // Callback Base waypoints
    sub_base_waypoints = nh.subscribe("base_waypoints", 10, &TrajectoryFollowing::cbbase_waypoints, this);
    /// Callback TwistStamped initialization
    sub_current_velocity = nh.subscribe("current_velocity", 10, &TrajectoryFollowing::cbcurrent_velocity, this);
    return true;
}

