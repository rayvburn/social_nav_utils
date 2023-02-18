#include <gtest/gtest.h>

#include <social_nav_utils/heading_direction_disturbance.h>

using namespace social_nav_utils;

TEST(TestHeadingDirection, scales) {
	double x_human = 0.0500;
	double y_human = -0.9500;
	double yaw_human = 0.3491;
	double cov_xx_human = 0.0856;
	double cov_xy_human = 0.0298;
	double cov_yy_human = 0.0145;
	double x_robot = 0.00;
	double y_robot = 0.1000;
	double yaw_robot = -1.6232;
	double vx_robot = 0.55;
	double vy_robot = 0.00;
	double occupancy_model_radius = 0.28;
	double fov = 3.6652;
	double circumradius_robot = 0.275;

	HeadingDirectionDisturbance hdd1(
		x_human,
		y_human,
		yaw_human,
		cov_xx_human,
		cov_xy_human,
		cov_yy_human,
		x_robot,
		y_robot,
		yaw_robot,
		vx_robot,
		vy_robot,
		occupancy_model_radius,
		fov
	);

	EXPECT_NEAR(hdd1.getDirectionScale(), 2.89681, 1e-03);
	EXPECT_NEAR(hdd1.getFovScale(), 0.167021969930533, 1e-03);
	EXPECT_DOUBLE_EQ(hdd1.getSpeedScale(), 0.55);
	EXPECT_NEAR(hdd1.getDistScale(), 1.0512, 1e-03);

	hdd1.normalize(circumradius_robot, vx_robot);

	// approximates
	EXPECT_NEAR(hdd1.getDirectionScale(), 2.89681 / 3.06338090001776, 1e-03);
	EXPECT_NEAR(hdd1.getFovScale(), 0.167021969930533 / 0.435383256331559, 1e-03);
	EXPECT_NEAR(hdd1.getSpeedScale(), vx_robot / vx_robot, 1e-03);
	EXPECT_NEAR(hdd1.getDistScale(), 1.0512 / (occupancy_model_radius + circumradius_robot), 1e-03);

	x_human = 0.2500;
	y_human = -0.9500;
	HeadingDirectionDisturbance hdd2(
		x_human,
		y_human,
		yaw_human,
		cov_xx_human,
		cov_xy_human,
		cov_yy_human,
		x_robot,
		y_robot,
		yaw_robot,
		vx_robot,
		vy_robot,
		occupancy_model_radius,
		fov
	);
	EXPECT_NEAR(hdd2.getDirectionScale(), 1.91154, 1e-03);
	EXPECT_NEAR(hdd2.getFovScale(), 0.122681402581972, 1e-03);
	EXPECT_DOUBLE_EQ(hdd2.getSpeedScale(), 0.55);
	EXPECT_NEAR(hdd2.getDistScale(), 1.07935165724615, 1e-03);
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
