#include <social_nav_utils/gaussians.h>

#include <angles/angles.h>

#include <cmath>
#include <math.h>

namespace social_nav_utils {

double calculateGaussian(double x, double mean, double variance, bool normalize) {
	double scale = 1.0;
	// with normalization, maximum possible value will be 1.0; otherwise, it depends on the value of variance
	if (!normalize) {
		scale = 1.0 / (std::sqrt(variance) * std::sqrt(2 * M_PI));
	}
	return scale * std::exp(-std::pow(x - mean, 2) / (2.0 * variance));
}

double calculateGaussianAngle(double x, double mean, double variance, bool normalize) {
	double gaussian1 = calculateGaussian(x, mean             , variance, normalize);
	double gaussian2 = calculateGaussian(x, mean - 2.0 * M_PI, variance, normalize);
	double gaussian3 = calculateGaussian(x, mean + 2.0 * M_PI, variance, normalize);
	return std::max(std::max(gaussian1, gaussian2), gaussian3);
}

double calculateGaussianAsymmetrical(
	double x,
	double y,
	double x_center,
	double y_center,
	double yaw,
	double variance_h,
	double variance_r,
	double variance_s
) {
	double sigma_h = std::sqrt(variance_h);
	double sigma_r = std::sqrt(variance_r);
	double sigma_s = std::sqrt(variance_s);

	double alpha = std::atan2(y - y_center, x - x_center) - yaw + M_PI_2;
	alpha = angles::normalize_angle(alpha);
	double sigma = (alpha <= 0.0 ? sigma_r : sigma_h);

	// save values used multiple times in computations;
	// squared cosine/sine of theta (yaw angle)
	double cos_yaw_sq = std::pow(std::cos(yaw), 2);
	double sin_yaw_sq = std::pow(std::sin(yaw), 2);
	double sin_2yaw = std::sin(2.0 * yaw);
	double sigma_sq = std::pow(sigma, 2);
	double sigma_s_sq = std::pow(sigma_s, 2);

	double a = cos_yaw_sq / (2.0 * sigma_sq) + sin_yaw_sq / (2.0 * sigma_s_sq);
	double b = sin_2yaw   / (4.0 * sigma_sq) - sin_2yaw   / (4.0 * sigma_s_sq);
	double c = sin_yaw_sq / (2.0 * sigma_sq) + cos_yaw_sq / (2.0 * sigma_s_sq);

	double exp_arg_a = a * (std::pow(x - x_center, 2));
	double exp_arg_b = 2.0 * b * (x - x_center) * (y - y_center);
	double exp_arg_c = c * std::pow(y - y_center, 2);
	return std::exp(-(exp_arg_a + exp_arg_b + exp_arg_c));
}

double calculateGaussian(const Eigen::VectorXd& x, const Eigen::VectorXd& mean, const Eigen::MatrixXd& cov) {
	calculateGaussian(x, mean, cov, x.rows());
}

double calculateGaussian(const Vector2d& x, const Vector2d& mean, const Matrix2d& cov) {
	calculateGaussian(x, mean, cov, 2.0);
}

double calculateGaussianAsymmetrical(
	const Eigen::VectorXd& x,
	const Eigen::VectorXd& mean,
	double mean_orientation,
	const Eigen::MatrixXd& cov_front,
	const Eigen::MatrixXd& cov_rear,
	bool unify_cov_scale
) {
	// can't tell if front or rear for 1D
	assert(x.size() > 1);
	return calculateGaussianAsymmetrical(x, mean, mean_orientation, cov_front, cov_rear, unify_cov_scale);
}

} // namespace social_nav_utils
