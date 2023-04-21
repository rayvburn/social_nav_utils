#include <gtest/gtest.h>

#include <social_nav_utils/formation_space_intrusion.h>

using namespace social_nav_utils;

TEST(TestMetricGaussian, formationSpaceGaussian) {
	// Based on Matlab implementation
	double gaussian1 = FormationSpaceIntrusion::computeFormationSpaceGaussian(
		2.0000, /* ospace_pos_x */
		2.7500, /* ospace_pos_y */
		0.0, /* ospace_orientation */
		0.255208333333333, /* ospace_variance_x */
		0.765625000000000, /* ospace_variance_y */
		0.427649644158897, /* pos_center_variance_xx */
		0.0, /* pos_center_variance_xyyx */
		0.487649597818208, /* pos_center_variance_yy */
		2.10000000000000, /* robot_pos_x */
		2.85000000000000 /* robot_pos_y */
	);
	EXPECT_NEAR(gaussian1, 0.170105832109089, 1e-05);

	double gaussian2 = FormationSpaceIntrusion::computeFormationSpaceGaussian(
		2.0000, /* ospace_pos_x */
		2.7500, /* ospace_pos_y */
		0.0, /* ospace_orientation */
		0.255208333333333, /* ospace_variance_x */
		0.765625000000000, /* ospace_variance_y */
		0.427649644158897, /* pos_center_variance_xx */
		0.0, /* pos_center_variance_xyyx */
		0.487649597818208, /* pos_center_variance_yy */
		3.30000000000000, /* robot_pos_x */
		4.05000000000000 /* robot_pos_y */
	);
	EXPECT_NEAR(gaussian2, 0.0254331283458186, 1e-05);

	/*
	* Matlab:
	* ellipseGeometricToStandard():
	*   x0 = 4.0;
	*   y0 = 2.0;
	*   aaxis = 1.0;
	*   baxis = 0.5;
	*   phi = -pi/6;
	* cov_fformation_uncert = [0.87654, 0.67892; 0.67892, 1.09876];
	*/
	double gaussian3 = FormationSpaceIntrusion::computeFormationSpaceGaussian(
		4.0000, /* ospace_pos_x */
		2.0000, /* ospace_pos_y */
		-0.523598775598299, /* ospace_orientation */
		0.250000000000000, /* ospace_variance_x */
		0.062500000000000, /* ospace_variance_y */
		0.87654, /* pos_center_variance_xx */
		0.67892, /* pos_center_variance_xyyx */
		1.09876, /* pos_center_variance_yy */
		3.45, /* robot_pos_x */
		1.75  /* robot_pos_y */
	);
	EXPECT_NEAR(gaussian3, 0.141921, 1e-05);
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
