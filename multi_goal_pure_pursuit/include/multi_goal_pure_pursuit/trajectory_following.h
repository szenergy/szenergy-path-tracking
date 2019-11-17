#ifndef TRAJECTORY_FOLLOWING_H
#define TRAJECTORY_FOLLOWING_H

#include <ros/ros.h>
/// Generated: ROS msgs
#include <geometry_msgs/PoseStamped.h>
#include <autoware_msgs/ControlCommandStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <autoware_msgs/LaneArray.h>
#include <std_msgs/Int32.h>
#include <autoware_msgs/LaneArray.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>

// Local includes
#include <multi_goal_pure_pursuit/multi_goal_state.h>

struct TrajectoryErrorHandling
{
	unsigned int cnt_error_tf_connectivity;

};

class TrajectoryFollowing
{
protected:
	ros::NodeHandle &nh;
	/// Topic publishers
    ros::Publisher pub_ctrl_cmd;
    /// Topic subscribers
    ros::Subscriber sub_current_lane_id;
    ros::Subscriber sub_closest_waypoints;
    ros::Subscriber sub_current_pose;
    ros::Subscriber sub_final_waypoints;
    ros::Subscriber sub_base_waypoints;
    ros::Subscriber sub_current_velocity;

    // State
    std::shared_ptr<TrajectoryFollowingState> state;
	// State machine
	SyncState sync_state;
public:
    TrajectoryFollowing(ros::NodeHandle& nh): nh(nh),
		state(new TrajectoryFollowingState()),
		sync_state(START)
	{
		
	}
	
	virtual bool init();
	/// Callback Int32
	void cbcurrent_lane_id(const std_msgs::Int32::ConstPtr &msg);
	/// Callback Int32
	void cbclosest_waypoints(const std_msgs::Int32::ConstPtr &msg);
	/// Callback PoseStamped
	void cbcurrent_pose(const geometry_msgs::PoseStamped::ConstPtr &msg);
	/// Callback LaneArray
	void cbfinal_waypoints(const autoware_msgs::Lane::ConstPtr &msg);
	/// Callback LaneArray
	void cbbase_waypoints(const autoware_msgs::Lane::ConstPtr &msg);
	/// Callback TwistStamped
	void cbcurrent_velocity(const geometry_msgs::TwistStamped::ConstPtr &msg);
	
};



#endif

