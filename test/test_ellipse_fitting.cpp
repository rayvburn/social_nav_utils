#include <gtest/gtest.h>

#include <social_nav_utils/ellipse_fitting.h>

#include <vector>

#include <ctime>
#include <iostream>

using namespace social_nav_utils;

// Expose protected
class EllipseFittingFallbackTest: public EllipseFitting {
public:
	EllipseFittingFallbackTest(): EllipseFitting(std::vector<double>{0.0}, std::vector<double>{0.0}) {}

	bool fitFallbackSingle(double x, double y) {
		return EllipseFitting::fitFallbackSingle(x, y);
	}

	bool fitFallbackMultiple(const std::vector<double>& x, std::vector<double>& y) {
		return EllipseFitting::fitFallbackMultiple(x, y);
	}
};

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

TEST(EllipseFitting, fallbackMultiple1) {
	auto X = std::vector<double>{0.5,  2.0, -2.0, -3.0, -1.0};
	auto Y = std::vector<double>{1.0, -2.5,  0.0,  3.0,  1.0};

	EllipseFittingFallbackTest ellip;
	// ignore default flow of fitting, we want to check fallback
	ellip.fitFallbackMultiple(X, Y);
	// NOTE: results from equivalent implementation in Matlab
	ASSERT_NEAR(ellip.getCenterX(), -0.7, 1e-03);
	ASSERT_NEAR(ellip.getCenterY(),  0.5, 1e-03);
	ASSERT_NEAR(ellip.getSemiAxisMajor(), 3.716517, 1e-03);
	ASSERT_NEAR(ellip.getSemiAxisMinor(), 1.261261, 1e-03);
	ASSERT_NEAR(ellip.getOrientation(), -0.83298127, 1e-03);
}

TEST(EllipseFitting, fallbackMultiple2) {
	auto X = std::vector<double>{0.5, 2.0, -2.0};
	auto Y = std::vector<double>{1.0, -2.5, 0.0};

	EllipseFittingFallbackTest ellip;
	// ignore default flow of fitting, we want to check fallback
	ellip.fitFallbackMultiple(X, Y);
	// NOTE: results from equivalent implementation in Matlab
	ASSERT_NEAR(ellip.getCenterX(), 0.166667, 1e-03);
	ASSERT_NEAR(ellip.getCenterY(),  -0.5, 1e-03);
	ASSERT_NEAR(ellip.getSemiAxisMajor(), 2.358495, 1e-03);
	ASSERT_NEAR(ellip.getSemiAxisMinor(), 1.086498, 1e-03);
	ASSERT_NEAR(ellip.getOrientation(), -0.55859932, 1e-03);
}

TEST(EllipseFitting, fallbackMultiple3) {
	auto X = std::vector<double>{0.1, 1.0, -2.0};
	auto Y = std::vector<double>{0.0, -2.5, -1.0};

	EllipseFittingFallbackTest ellip;
	// ignore default flow of fitting, we want to check fallback
	ellip.fitFallbackMultiple(X, Y);
	// NOTE: results from equivalent implementation in Matlab
	ASSERT_NEAR(ellip.getCenterX(), -0.3, 1e-03);
	ASSERT_NEAR(ellip.getCenterY(), -1.166667, 1e-03);
	ASSERT_NEAR(ellip.getSemiAxisMajor(), 1.677051, 1e-03);
	ASSERT_NEAR(ellip.getSemiAxisMinor(), 0.916788, 1e-03);
	ASSERT_NEAR(ellip.getOrientation(), -0.46364761, 1e-03);
}

TEST(EllipseFitting, fallbackMultiple4) {
	auto X = std::vector<double>{0.1, 1.0};
	auto Y = std::vector<double>{0.0, -2.5};

	EllipseFittingFallbackTest ellip;
	// ignore default flow of fitting, we want to check fallback
	ellip.fitFallbackMultiple(X, Y);
	// NOTE: results from equivalent implementation in Matlab
	ASSERT_NEAR(ellip.getCenterX(), 0.55, 1e-03);
	ASSERT_NEAR(ellip.getCenterY(), -1.25, 1e-03);
	ASSERT_NEAR(ellip.getSemiAxisMajor(), 1.328533, 1e-03);
	ASSERT_NEAR(ellip.getSemiAxisMinor(), 0.28, 1e-03);
	ASSERT_NEAR(ellip.getOrientation(), -1.22524075, 1e-03);
}

TEST(EllipseFitting, fallbackMultiple5) {
	auto X = std::vector<double>{-2.5, -1.5};
	auto Y = std::vector<double>{1.5, -2.5};

	EllipseFittingFallbackTest ellip;
	// ignore default flow of fitting, we want to check fallback
	ellip.fitFallbackMultiple(X, Y);
	// NOTE: results from equivalent implementation in Matlab
	ASSERT_NEAR(ellip.getCenterX(), -2.0, 1e-03);
	ASSERT_NEAR(ellip.getCenterY(), -0.5, 1e-03);
	ASSERT_NEAR(ellip.getSemiAxisMajor(), 2.061553, 1e-03);
	ASSERT_NEAR(ellip.getSemiAxisMinor(), 0.28, 1e-03);
	ASSERT_NEAR(ellip.getOrientation(), -1.32581766, 1e-03);
}

TEST(EllipseFitting, fallbackMultiple6) {
	auto X = std::vector<double>{-2.0, -2.0, -2.0};
	auto Y = std::vector<double>{1.5, -2.5, -2.0};

	EllipseFittingFallbackTest ellip;
	// ignore default flow of fitting, we want to check fallback
	ellip.fitFallbackMultiple(X, Y);
	// NOTE: results from equivalent implementation in Matlab
	ASSERT_NEAR(ellip.getCenterX(), -2.0, 1e-03);
	ASSERT_NEAR(ellip.getCenterY(), -1.0, 1e-03);
	ASSERT_NEAR(ellip.getSemiAxisMajor(), 2.0, 1e-03);
	ASSERT_NEAR(ellip.getSemiAxisMinor(), 0.28, 1e-03);
	ASSERT_NEAR(ellip.getOrientation(), 1.57079633, 1e-03);
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
