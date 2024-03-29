#include <gtest/gtest.h>

#include <social_nav_utils/personal_space_intrusion.h>

using namespace social_nav_utils;

TEST(TestMetricGaussian, personalSpaceGaussian) {
	// Based on Matlab implementation: check pos, pcov, variance_*, then after computations look at X3mv, Y3mv and G3mv
	double gaussian1 = PersonalSpaceIntrusion::computePersonalSpaceGaussian(
		3.13779826339, /* person_pos_x */
		5.19603112355, /* person_pos_y */
		2.53422628911270, /* person_orient_yaw */
		0.127649644158897, /* person_pos_cov_xx */
		-9.01197659836358e-08, /* person_pos_cov_xy */
		-9.01197659853060e-08, /* person_pos_cov_yx */
		0.127649597818208, /* person_pos_cov_yy */
		3.00, /* person_ps_var_front */
		0.75, /* person_ps_var_rear */
		1.33, /* person_ps_var_side */
		2.68779826339000, /* robot_pos_x */
		4.74603112355000, /* robot_pos_y */
		true /* unify_asymmetry_scale */
	);
	EXPECT_NEAR(gaussian1, 0.122746777488856, 1e-05);

	double gaussian2 = PersonalSpaceIntrusion::computePersonalSpaceGaussian(
		3.13779826339, /* person_pos_x */
		5.19603112355, /* person_pos_y */
		2.53422628911270, /* person_orient_yaw */
		0.127649644158897, /* person_pos_cov_xx */
		-9.01197659836358e-08, /* person_pos_cov_xy */
		-9.01197659853060e-08, /* person_pos_cov_yx */
		0.127649597818208, /* person_pos_cov_yy */
		3.00, /* person_ps_var_front */
		0.75, /* person_ps_var_rear */
		1.33, /* person_ps_var_side */
		3.03779826339000, /* robot_pos_x */
		8.24603112355000, /* robot_pos_y */
		true /* unify_asymmetry_scale */
	);
	EXPECT_NEAR(gaussian2, 0.0106003940784025, 1e-05);

	/*
	 * pos = [1.123; 7.321; 0.0];
	 * quat = quaternion(0, 0, 0.1719802, 0.9851004);
	 * pcov(1,1) = 1.321654;
	 * pcov(1,2) = 0.456321;
	 * pcov(2,1) = 0.456321;
	 * pcov(2,2) = 0.321654;
	 * variance_h = 2.00;
	 * variance_r = 0.50;
	 * variance_s = 1.00;
	 */
	double gaussian3 = PersonalSpaceIntrusion::computePersonalSpaceGaussian(
		1.123, /* person_pos_x */
		7.321, /* person_pos_y */
		0.345678938849738, /* person_orient_yaw */
		1.321654, /* person_pos_cov_xx */
		0.456321, /* person_pos_cov_xy */
		0.456321, /* person_pos_cov_yx */
		0.321654, /* person_pos_cov_yy */
		2.00, /* person_ps_var_front */
		0.50, /* person_ps_var_rear */
		1.00, /* person_ps_var_side */
		1.173, /* robot_pos_x */
		7.821, /* robot_pos_y */
		true /* unify_asymmetry_scale */
	);
	EXPECT_NEAR(gaussian3, 0.0956295, 1e-05);
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
