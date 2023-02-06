#include <gtest/gtest.h>

#include <social_nav_utils/relative_location.h>

using namespace social_nav_utils;

TEST(TestRelativeLocation, example1) {
	RelativeLocation rl(2.0, 2.0, M_PI_2, 3.0, 3.0);
	ASSERT_DOUBLE_EQ(rl.getAngle(), -M_PI_4);
	ASSERT_TRUE(rl.isFront());
	ASSERT_FALSE(rl.isLeftSide());
}

TEST(TestRelativeLocation, example2) {
	RelativeLocation rl(2.0, 2.0, M_PI_2, 1.0, 1.0);
	ASSERT_DOUBLE_EQ(rl.getAngle(), 3.0 * M_PI_4);
	ASSERT_FALSE(rl.isFront());
	ASSERT_TRUE(rl.isLeftSide());
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
