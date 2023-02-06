#include <gtest/gtest.h>

#include <social_nav_utils/heading_direction_disturbance.h>
#include <math.h>

using namespace social_nav_utils;

TEST(TestHeadingDirection, comparison) {
	HeadingDirectionDisturbance hdd_cc(
		1.0, 1.0, M_PI_2, M_PI,
		3.0, 3.0, -3 * M_PI_4,
		1.25, 0.05
	);

	HeadingDirectionDisturbance hdd_outwards(
		1.0, 1.0, M_PI_2, M_PI,
		3.0, 3.0, M_PI_4,
		1.25, 0.05
	);
	EXPECT_LT(hdd_outwards.getDirectionScale(), 1e-03);

	HeadingDirectionDisturbance hdd_cf(
		1.0, 1.0, M_PI_2, M_PI,
		3.0, 3.0, M_PI_4,
		1.25, 0.05
	);
	EXPECT_LT(hdd_cf.getDirectionScale(), hdd_cc.getDirectionScale());
	EXPECT_GT(hdd_cf.getDirectionScale(), hdd_outwards.getDirectionScale());
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
