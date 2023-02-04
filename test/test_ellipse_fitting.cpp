#include <gtest/gtest.h>

#include <social_nav_utils/ellipse_fitting.h>

#include <vector>

#include <ctime>
#include <iostream>

using namespace social_nav_utils;

TEST(EllipseFitting, solverTest1) {
	auto X = std::vector<double>{1.0, 2.0, 3.0, 2.0};
	auto Y = std::vector<double>{3.0, 4.5, 3.0, 1.0};

	EllipseFitting ellip(X, Y);

	ASSERT_FALSE(ellip.usedFallback());
	ASSERT_NEAR(ellip.getCenterX(), 2, 1e-03);
	ASSERT_NEAR(ellip.getCenterY(), 2.75, 1e-03);
	ASSERT_NEAR(ellip.getSemiAxisMajor(), 1.75, 1e-03);
	ASSERT_NEAR(ellip.getSemiAxisMinor(), 1.01036, 1e-03);
	ASSERT_NEAR(ellip.getOrientation(), -1.5708, 1e-03);
}

TEST(EllipseFitting, DISABLED_solverTest2) {
	auto X = std::vector<double>{1.0, 2.0, 3.0, 2.0};
	auto Y = std::vector<double>{3.0, 4.0, 3.0, 1.0};
}

TEST(EllipseFitting, solverTest3) {
	auto X = std::vector<double>{1.0, 2.0, 3.0, 2.0};
	auto Y = std::vector<double>{3.0, 4.0, 3.0, 1.0};

	EllipseFitting ellip(X, Y);

	ASSERT_FALSE(ellip.usedFallback());
	ASSERT_NEAR(ellip.getCenterX(), 2, 1e-03);
	ASSERT_NEAR(ellip.getCenterY(), 2.5, 1e-03);
	ASSERT_NEAR(ellip.getSemiAxisMajor(), 1.5, 1e-03);
	ASSERT_NEAR(ellip.getSemiAxisMinor(), 1.06066, 1e-03);
	ASSERT_NEAR(ellip.getOrientation(), 1.5708, 1e-03);
}

TEST(EllipseFitting, solverTest4) {
	auto X = std::vector<double>{1.0, 3.0, 2.0};
	auto Y = std::vector<double>{3.0, 3.0, 1.0};

	EllipseFitting ellip(X, Y);

	ASSERT_TRUE(ellip.usedFallback());
}

TEST(EllipseFitting, solverTest5) {
	// difficult case
	auto X = std::vector<double>{1.0, 2.0};
	auto Y = std::vector<double>{3.0, 1.0};

	EllipseFitting ellip(X, Y);

	ASSERT_TRUE(ellip.usedFallback());
}

TEST(EllipseFitting, single) {
	auto X = std::vector<double>{1.25};
	auto Y = std::vector<double>{1.35};

	EllipseFitting ellip(X, Y);

	ASSERT_TRUE(ellip.usedFallback());
	ASSERT_DOUBLE_EQ(ellip.getCenterX(), 1.25);
	ASSERT_DOUBLE_EQ(ellip.getCenterY(), 1.35);
	ASSERT_NE(ellip.getSemiAxisMajor(), NAN);
	ASSERT_NE(ellip.getSemiAxisMinor(), NAN);
	ASSERT_DOUBLE_EQ(ellip.getOrientation(), 0.0);
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
