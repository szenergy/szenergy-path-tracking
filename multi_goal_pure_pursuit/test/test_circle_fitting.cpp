/*
 * test_circle_fitting.cpp
 *
 *  Created on: Nov 25, 2019
 *      Author: kyberszittya
 */

#include <gtest/gtest.h>

#include <multi_goal_pure_pursuit/multi_goal_pure_pursuit.h>


TEST(TestBasicCircleFitting, TestOnePoint)
{
	autoware_msgs::Lane lane;
	geometry_msgs::Point start;
	start.x = 0.0;
	autoware_msgs::Waypoint wp0;
	wp0.pose.pose.position.x = 0.0;
	wp0.pose.pose.position.y = 2.0;
	wp0.pose.pose.position.z = 0.0;
	lane.waypoints.push_back(wp0);
	autoware_msgs::Waypoint wp1;
	wp1.pose.pose.position.x = 1.0;
	wp1.pose.pose.position.y = 1.0;
	wp1.pose.pose.position.z = 0.0;
	lane.waypoints.push_back(wp1);
	autoware_msgs::Waypoint wp2;
	wp2.pose.pose.position.x = -1.0;
	wp2.pose.pose.position.y = 1.0;
	wp2.pose.pose.position.z = 0.0;
	lane.waypoints.push_back(wp2);
	std::cout << "Simple circle fitting with one point" << '\n';
	std::cout << "Radius:" << MultiGoalPurePursuit::calcLsRadius(3, 0, start, lane) << '\n';
}

int main(int argc, char** argv)
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
