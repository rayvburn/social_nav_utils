#include <gtest/gtest.h>

#include <social_nav_utils/passing_speed_comfort.h>

using namespace social_nav_utils;

TEST(TestPassingSpeedComfort, farDistances) {
	// expected results based on Matlab implementation
	const double DIST_FAR = 1.0;
	EXPECT_NEAR(PassingSpeedComfort::computeSpeedComfort(DIST_FAR, 0.35), 5.9684, 1e-03);
	EXPECT_NEAR(PassingSpeedComfort::computeSpeedComfort(DIST_FAR, 0.50), 6.1627, 1e-03);
	EXPECT_NEAR(PassingSpeedComfort::computeSpeedComfort(DIST_FAR, 0.90), 6.1433, 1e-03);
}

TEST(TestPassingSpeedComfort, closeDistances) {
	// expected results based on Matlab implementation
	const double DIST_CLOSE = 0.5;
	EXPECT_NEAR(PassingSpeedComfort::computeSpeedComfort(DIST_CLOSE, 0.35), 5.2103, 1e-03);
	EXPECT_NEAR(PassingSpeedComfort::computeSpeedComfort(DIST_CLOSE, 0.50), 5.0931, 1e-03);
	EXPECT_NEAR(PassingSpeedComfort::computeSpeedComfort(DIST_CLOSE, 0.90), 4.6429, 1e-03);
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
