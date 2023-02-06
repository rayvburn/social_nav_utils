#include <gtest/gtest.h>

#include <social_nav_utils/distance_vector.h>

using namespace social_nav_utils;

TEST(TestDistanceVector, example1) {
	DistanceVector d(1.0, 1.0, 2.0, 2.0);
	ASSERT_DOUBLE_EQ(d.getLength(), std::sqrt(2));
	ASSERT_DOUBLE_EQ(d.getAngle(), M_PI_4);
	// direction is important here
	ASSERT_DOUBLE_EQ(d.getX(), 1.0);
	ASSERT_DOUBLE_EQ(d.getY(), 1.0);
}

TEST(TestDistanceVector, example2) {
	DistanceVector d(2.0, 2.0, -2.0, -2.0);
	ASSERT_DOUBLE_EQ(d.getLength(), 4.0 * std::sqrt(2));
	ASSERT_DOUBLE_EQ(d.getAngle(), -3.0 * M_PI_4);
	// direction is important here
	ASSERT_DOUBLE_EQ(d.getX(), -4.0);
	ASSERT_DOUBLE_EQ(d.getY(), -4.0);
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
