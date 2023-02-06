#include <gtest/gtest.h>

#include <social_nav_utils/gaussians.h>

using namespace social_nav_utils;

TEST(TestGaussians, univariate) {
	// Matlab: normpdf(0, 0, 1)
	auto g1 = calculateGaussian(0.0, 0.0, std::pow(1.0, 2));
	ASSERT_NEAR(g1, 0.3989, 1e-03);

	// Matlab: normpdf(0, 0, 2)
	auto g2 = calculateGaussian(0.0, 0.0, std::pow(2.0, 2));
	ASSERT_NEAR(g2, 0.1995, 1e-03);

	// always 1.0 at mean
	auto g3 = calculateGaussian(0.0, 0.0, std::pow(3.0, 2), true);
	ASSERT_DOUBLE_EQ(g3, 1.0);
}

TEST(TestGaussians, univariateAngleDomain) {
	auto g1 = calculateGaussianAngle(M_PI, -M_PI, std::pow(1.0, 2));
	ASSERT_NEAR(g1, 0.3989, 1e-03);

	auto g2 = calculateGaussianAngle(-M_PI, M_PI, std::pow(2.0, 2));
	ASSERT_NEAR(g2, 0.1995, 1e-03);

	auto g3 = calculateGaussianAngle(-M_PI, M_PI, std::pow(3.0, 2), true);
	ASSERT_DOUBLE_EQ(g3, 1.0);
}

TEST(TestGaussians, calculateGaussianAsymmetrical) {
	// despite huge standard deviations, the value at mean will be 1
	auto g = calculateGaussianAsymmetrical(0.0, 0.0, 0.0, 0.0, 0.0, 1e06, 1e06, 1e06);
	ASSERT_DOUBLE_EQ(g, 1.0);
}

// Ref: http://blog.sarantop.com/notes/mvn
TEST(TestGaussians, multivariateMatrixForm) {
	// Define the covariance matrix, the mean and test state
	Eigen::MatrixXd cov(2, 2);
	cov <<  1, 0.1,
			0.1, 1;
	Eigen::VectorXd mean(2);
	mean << 0, 0;
	Eigen::VectorXd test(2);
	test << 0, 0;
	EXPECT_NEAR(calculateGaussian(test, mean, cov), 0.16, 1e-4);

	// different state for testing
	test << -0.6, -0.6;
	EXPECT_NEAR(calculateGaussian(test, mean, cov), 0.1153, 1e-4);
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
