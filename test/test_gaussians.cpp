#include <gtest/gtest.h>

#include <social_nav_utils/gaussians.h>

using namespace social_nav_utils;

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
