/*
 * test_basic_functions.cpp
 *
 *  Created on: Nov 7, 2019
 *      Author: kyberszittya
 */

#include <gtest/gtest.h>

#include <multi_goal_pure_pursuit/multi_goal_pure_pursuit.h>

TEST(TestBasic, TestDistanceBetweenPoses_ZeroDistance)
{
	geometry_msgs::Pose pose0;
	pose0.position.x = 1.0;
	pose0.position.y = 1.0;
	pose0.position.z = 1.0;
	geometry_msgs::Pose pose1;
	pose1.position.x = 1.0;
	pose1.position.y = 1.0;
	pose1.position.z = 1.0;
	double d = distanceBetweenPoses(pose0, pose1);
	ASSERT_DOUBLE_EQ(0.0, d);
}

TEST(TestBasic, TestDistanceBetweenPoses_SomeDistance)
{
	geometry_msgs::Pose pose0;
	pose0.position.x = 0.0;
	pose0.position.y = 0.0;
	pose0.position.z = 1.0;
	geometry_msgs::Pose pose1;
	pose1.position.x = 2.0;
	pose1.position.y = 2.0;
	pose1.position.z = 1.0;
	double d = distanceBetweenPoses(pose0, pose1);
	ASSERT_DOUBLE_EQ(sqrt(8), d);
}

TEST(TestBasic, TestDistanceBetweenPoses_SomeDistance3D)
{
	geometry_msgs::Pose pose0;
	pose0.position.x = 0.0;
	pose0.position.y = 0.0;
	pose0.position.z = 0.0;
	geometry_msgs::Pose pose1;
	pose1.position.x = 2.0;
	pose1.position.y = 2.0;
	pose1.position.z = 1.0;
	double d = distanceBetweenPoses(pose0, pose1);
	ASSERT_DOUBLE_EQ(3.0, d);
}

TEST(TestBasic, PlaneDistance2D_SomeDistance)
{
	geometry_msgs::Point p0;
	p0.x = 0.0;
	p0.y = 0.0;
	p0.z = 0.0;
	geometry_msgs::Point p1;
	p1.x = 2.0;
	p1.y = 2.0;
	p1.z = 1.0;
	double d = get2DPlaneDistance(p0, p1);
	ASSERT_DOUBLE_EQ(sqrt(8), d);
}

TEST(TestBasic, LineDistance2DToPoint)
{
	geometry_msgs::Point p0;
	p0.x = 0.0;
	p0.y = 0.0;
	p0.z = 0.0;
	geometry_msgs::Point p1;
	p1.x = 0.0;
	p1.y = 2.0;
	p1.z = 0.0;
	geometry_msgs::Point state;
	state.x = -2.0;
	state.y = 1.0;
	state.z = 0.0;
	double distance = linearInterpolationDistance2D(p0, p1, state);
	ASSERT_DOUBLE_EQ(2.0, distance);
}

TEST(TestBasic, LineDistance2DToPointAlternatingHeight)
{
	geometry_msgs::Point p0;
	p0.x = 0.0;
	p0.y = 0.0;
	p0.z = 5.0;
	geometry_msgs::Point p1;
	p1.x = 0.0;
	p1.y = 2.0;
	p1.z = 8.0;
	geometry_msgs::Point state;
	state.x = -2.0;
	state.y = 1.0;
	state.z = 6.0;
	double distance = linearInterpolationDistance2D(p0, p1, state);
	ASSERT_DOUBLE_EQ(2.0, distance);
}

const int MAX_STEPS_SEMGENTS = 10;

TEST(TestBasic, LaneArrayIntegration)
{
	autoware_msgs::Lane lane;
	std::cout << "Calc integrate of the following function" << '\n';
	std::cout << "|    /\\" << '\n';
	std::cout << "|   /  \\" << '\n';
	std::cout << "|  /    \\" << '\n';
	std::cout << "| /      \\" << '\n';
	std::cout << "|/________\\____________" << '\n';
	std::cout << "|          \\        /" << '\n';
	std::cout << "|           \\      /" << '\n';
	std::cout << "|            \\    /" << '\n';
	std::cout << "|             \\  /" << '\n';
	std::cout << "|              \\/" << '\n';
	std::cout << "Area is zero" << '\n';
	for (int i = 0; i < MAX_STEPS_SEMGENTS; i++)
	{
		autoware_msgs::Waypoint wp;
		wp.pose.pose.position.y = i*0.5;
		lane.waypoints.push_back(std::move(wp));
	}
	for (int i = 0; i < 2 * MAX_STEPS_SEMGENTS; i++)
	{
		autoware_msgs::Waypoint wp;
		wp.pose.pose.position.y = -i*0.5 + MAX_STEPS_SEMGENTS*0.5;
		lane.waypoints.push_back(std::move(wp));
	}
	for (int i = 0; i < MAX_STEPS_SEMGENTS; i++)
	{
		autoware_msgs::Waypoint wp;
		wp.pose.pose.position.y = i*0.5 - MAX_STEPS_SEMGENTS*0.5;
		lane.waypoints.push_back(std::move(wp));
	}
	geometry_msgs::TransformStamped current_loc_vehicle;
	current_loc_vehicle.transform.translation.x = 0.0;
	current_loc_vehicle.transform.translation.y = 0.0;
	current_loc_vehicle.transform.translation.z = 0.0;
	current_loc_vehicle.transform.rotation.w = 1.0;
	current_loc_vehicle.transform.rotation.x = 0.0;
	current_loc_vehicle.transform.rotation.y = 0.0;
	current_loc_vehicle.transform.rotation.z = 0.0;
	double error = integrateLaneLateralError(
			current_loc_vehicle,
			lane, 0, 40);
	ASSERT_DOUBLE_EQ(error, 0.0);
}

int main(int argc, char** argv)
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
