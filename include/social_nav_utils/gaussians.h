#pragma once

#include <eigen3/Eigen/Core>

namespace social_nav_utils {

/**
 * Computes a value of univariate Gaussian probability density function
 *
 * @url https://en.wikipedia.org/wiki/Gaussian_function
 */
double calculateGaussian(double x, double mean, double variance, bool normalize = false);

/**
 * Computes value of univariate Gaussian PDF but includes wrapped regions of bell curve (shifted -2pi and +2pi)
 *
 * Note that with @ref normalize set to true, it no longer computes probability density function
 * as its integral won't sum up to 1
 */
double calculateGaussianAngle(double x, double mean, double variance, bool normalize = false);

/**
 * @brief Calculates value of bivariate asymmetrical Gaussian according to formulation proposed by Kirby
 *
 * Note that this method does not account for scale, no matter of variance, the value in the center will be 1.0,
 * so in fact this is not a Distribution.
 *
 * Reference: Algorithm A.1 from Kirby, 2010 PhD thesis "Social Robot Navigation" (p. 166)
 * @url https://www.ri.cmu.edu/pub_files/2010/5/rk_thesis.pdf
 */
double calculateGaussianAsymmetrical(
	double x,
	double y,
	double x_center,
	double y_center,
	double yaw,
	double variance_h,
	double variance_r,
	double variance_s
);

/**
 * @brief Computes a value of Gaussian described with mean vector and covariance matrix
 *
 * This is the most handy for multivariate Gaussians
 *
 * Based on http://blog.sarantop.com/notes/mvn
 */
double calculateGaussian(const Eigen::VectorXd& x, const Eigen::VectorXd& mean, const Eigen::MatrixXd& cov);

/**
 * @brief Computes a value of asymmetrical Gaussian described with a mean (at least 2 elem.) and 2 covariance matrices
 *
 * Evaluates whether x is located in front of the mean and selects appropriate covariance matrix to compute PDF
 *
 * @param x vector for whom the PDF is computed for
 * @param mean mean of the Gaussian
 * @param mean_orientation 'orientation' of the mean of the Gaussian (assuming that x does not contain this information)
 * @param cov_front covariance matrix for the front case
 * @param cov_rear covariance matrix for the rear case
 * @param unify_cov_scale adjusts scale of the output to avoid a step when front/rear covariances strongly differ
 */
double calculateGaussianAsymmetrical(
	const Eigen::VectorXd& x,
	const Eigen::VectorXd& mean,
	double mean_orientation,
	const Eigen::MatrixXd& cov_front,
	const Eigen::MatrixXd& cov_rear,
	bool unify_cov_scale = false
);

} // namespace social_nav_utils
