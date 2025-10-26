#include <gtest/gtest.h>

#include <social_nav_utils/heading_direction_disturbance.h>

using namespace social_nav_utils;

TEST(TestHeadingDirection, scales) {
	/*
	 * human pos:
	 *   xo + 0.05,...
	 *   yo - 0.95,...
	 *   deg2rad(+20)...
	 * robot pos:
	 *   xo - 0.00, ...
	 *   yo + 0.10, ...
	 *   deg2rad(-93)...
	 * human pos covariance (w/o rotation):
	 *   cov_h(1,1) = 0.0964960000000000;
	 *   cov_h(1,2) = 0.0000000030120000;
	 *   cov_h(2,1) = 0.0000000030120000;
	 *   cov_h(2,2) = 0.0036496000000000;
	 */
	double x_human = 0.0500;
	double y_human = -0.9500;
	double yaw_human = 0.349065850398866;
	double cov_xx_human = 0.0856350324557247;
	double cov_xy_human = 0.0298402600693260;
	double cov_yy_human = 0.0145105675442753;
	double x_robot = 0.000;
	double y_robot = 0.100;
	double yaw_robot = -1.62315620435473;

	double vx_robot = 0.55;
	double vy_robot = 0.00;
	double occupancy_model_radius = 0.28; // OCCUP_MODEL_SIZE
	double fov = 3.6652; // fov_human
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

	// stored in the `gaussian_result` variable
	EXPECT_NEAR(hdd1.getDirectionScale(), 2.87146315318844, 1e-06);
	// computed with `computeGaussianAngleDomain(dist_vec_angle - p_h(3), 0, fov_variance)`
	EXPECT_NEAR(hdd1.getFovScale(), 0.166793381244129, 1e-06);
	EXPECT_DOUBLE_EQ(hdd1.getSpeedScale(), 0.55);
	// stored in the `d_rh` variable
	EXPECT_NEAR(hdd1.getDistScale(), 1.05118980208143, 1e-06);

	hdd1.normalize(circumradius_robot, vx_robot);

	// approximates
	// maximum Gaussian computed with `MvGaussian(p_h(1:2), p_h(1:2), cov_sum(1:2, 1:2))`
	EXPECT_NEAR(hdd1.getDirectionScale(), 2.87146315318844 / 3.06334678136974, 1e-04);
	// maximum FOV computed with `computeGaussianAngleDomain(0, 0, fov_variance)`
	EXPECT_NEAR(hdd1.getFovScale(), 0.166793381244129 / 0.435383914003528, 1e-06);
	EXPECT_NEAR(hdd1.getSpeedScale(), vx_robot / vx_robot, 1e-06);
	EXPECT_NEAR(hdd1.getDistScale(), 1.05118980208143 / (occupancy_model_radius + circumradius_robot), 1e-06);

	/*
	 * human pos:
	 *   xo + 0.25,...
	 *   yo - 0.95,...
	 *   deg2rad(+20)...
	 */
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
	EXPECT_NEAR(hdd2.getDirectionScale(), 1.93938442094628, 1e-06);
	EXPECT_NEAR(hdd2.getFovScale(), 0.123307533540991, 1e-06);
	EXPECT_DOUBLE_EQ(hdd2.getSpeedScale(), 0.55);
	EXPECT_NEAR(hdd2.getDistScale(), 1.07935165724615, 1e-06);

	/*
	 * human pos:
	 *   xo + 0.15,...
	 *   yo - 0.75,...
	 *   deg2rad(+90)...
	 * robot pos:
	 *   xo - 0.25, ...
	 *   yo + 2.00, ...
	 *   deg2rad(-100)...
	 * human pos covariance:
	 *   cov_h(1,1) = 1.12345;
	 *   cov_h(1,2) = 0.65445;
	 *   cov_h(2,1) = 0.65445;
	 *   cov_h(2,2) = 1.25814;
	 */
	HeadingDirectionDisturbance hdd3(
		+0.15,
		-0.75,
		M_PI_2,
		+1.25814, // cov_xx_human
		-0.65445, // cov_xy_human
		+1.12345, // cov_yy_human
		-0.25,
		+2.00,
		-1.74532925199433,
		vx_robot,
		vy_robot,
		occupancy_model_radius,
		fov
	);
	EXPECT_NEAR(hdd3.getDirectionScale(), 0.0909645031495623, 1e-06);

	/*
	 * Human behind the robot
	 *
	 * human pos:
	 *   xo + 0.05,...
	 *   yo - 0.95,...
	 *   deg2rad(+90)...
	 * robot pos:
	 *   xo - 0.00, ...
	 *   yo + 0.10, ...
	 *   deg2rad(+95)...
	 * human pos covariance:
	 *   cov_h(1,1) = 1.98765;
	 *   cov_h(1,2) = 1.25891;
	 *   cov_h(2,1) = 1.25891;
	 *   cov_h(2,2) = 1.67891;
	 */
	HeadingDirectionDisturbance hdd4(
		+0.05,
		-0.95,
		M_PI_2,
		+1.67891, // cov_xx_human
		-1.25891, // cov_xy_human
		+1.98765, // cov_yy_human
		-0.00,
		+0.10,
		1.65806278939461,
		vx_robot,
		vy_robot,
		occupancy_model_radius,
		fov
	);
	EXPECT_NEAR(hdd4.getDirectionScale(), 0.0, 1e-06);

	/*
	 * Human behind the robot and the human's direction is parallel to the robot's
	 *
	 * human pos:
	 *   xo + 0.00,...
	 *   yo - 2.00,...
	 *   deg2rad(+90)...
	 * robot pos:
	 *   xo - 0.00, ...
	 *   yo + 0.00, ...
	 *   deg2rad(+90)...
	 * human pos covariance:
	 *   cov_h(1,1) = 0.09876;
	 *   cov_h(1,2) = 0.02589;
	 *   cov_h(2,1) = 0.02589;
	 *   cov_h(2,2) = 0.06789;
	 */
	HeadingDirectionDisturbance hdd5(
		+0.00,
		-2.00,
		M_PI_2,
		+0.06789, // cov_xx_human
		-0.02589, // cov_xy_human
		+0.09876, // cov_yy_human
		-0.00,
		+0.00,
		M_PI_2,
		vx_robot,
		vy_robot,
		occupancy_model_radius,
		fov
	);
	EXPECT_NEAR(hdd5.getDirectionScale(), 0.0, 1e-06);

	/*
	 * Human on the side of the robot and the human's direction parallel to the robot's
	 *
	 * human pos:
	 *   xo + 0.00,...
	 *   yo - 2.00,...
	 *   deg2rad(0)...
	 * robot pos:
	 *   xo - 0.00, ...
	 *   yo + 0.00, ...
	 *   deg2rad(0)...
	 * human pos covariance:
	 *   cov_h(1,1) = 0.009;
	 *   cov_h(1,2) = 0.002;
	 *   cov_h(2,1) = 0.002;
	 *   cov_h(2,2) = 0.006;
	 */
	HeadingDirectionDisturbance hdd6(
		+0.00,
		-2.00,
		0.0,
		+0.009, // cov_xx_human
		-0.002, // cov_xy_human
		+0.006, // cov_yy_human
		-0.00,
		+0.00,
		0.0,
		vx_robot,
		vy_robot,
		occupancy_model_radius,
		fov
	);
	EXPECT_NEAR(hdd6.getDirectionScale(), 0.0, 1e-06);

	/*
	 * Human on the side of the robot and the human's direction almost parallel to the robot's
	 *
	 * human pos:
	 *   xo + 0.00,...
	 *   yo - 2.00,...
	 *   deg2rad(0)...
	 * robot pos:
	 *   xo - 0.00, ...
	 *   yo + 0.00, ...
	 *   deg2rad(-5)...
	 * human pos covariance:
	 *   cov_h(1,1) = 0.009;
	 *   cov_h(1,2) = 0.002;
	 *   cov_h(2,1) = 0.002;
	 *   cov_h(2,2) = 0.006;
	 */
	HeadingDirectionDisturbance hdd7(
		+0.00,
		-2.00,
		0.0,
		+0.009, // cov_xx_human
		-0.002, // cov_xy_human
		+0.006, // cov_yy_human
		-0.00,
		+0.00,
		-0.0872664625997165,
		vx_robot,
		vy_robot,
		occupancy_model_radius,
		fov
	);
	EXPECT_NEAR(hdd7.getDirectionScale(), 0.0, 1e-06);

	/*
	 * Human on the side of the robot and the human's direction is perpendicular to the robot's
	 *
	 * human pos:
	 *   xo + 0.00,...
	 *   yo - 2.00,...
	 *   deg2rad(0)...
	 * robot pos:
	 *   xo - 0.00, ...
	 *   yo + 0.00, ...
	 *   deg2rad(90)...
	 * human pos covariance:
	 *   cov_h(1,1) = 0.009;
	 *   cov_h(1,2) = 0.002;
	 *   cov_h(2,1) = 0.002;
	 *   cov_h(2,2) = 0.006;
	 */
	HeadingDirectionDisturbance hdd8(
		+0.00,
		-2.00,
		0.0,
		+0.009, // cov_xx_human
		-0.002, // cov_xy_human
		+0.006, // cov_yy_human
		-0.00,
		+0.00,
		M_PI,
		vx_robot,
		vy_robot,
		occupancy_model_radius,
		fov
	);
	EXPECT_NEAR(hdd8.getDirectionScale(), 0.0, 1e-06);
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
