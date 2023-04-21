#pragma once

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
// may prevent compilation errors calling to matrix.inverse()
#include <eigen3/Eigen/LU>

// custom classes for linear algebra with API similar to Eigen
#include <social_nav_utils/math/core.h>

// used in template functions
#include <social_nav_utils/relative_location.h>

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
 * @brief @ref calculateGaussian template specialization
 */
double calculateGaussian(const Eigen::VectorXd& x, const Eigen::VectorXd& mean, const Eigen::MatrixXd& cov);

/**
 * @brief @ref calculateGaussian template specialization
 */
double calculateGaussian(const Vector2d& x, const Vector2d& mean, const Matrix2d& cov);

/**
 * @brief @ref calculateGaussianAsymmetrical template specialization
 */
double calculateGaussianAsymmetrical(
	const Eigen::VectorXd& x,
	const Eigen::VectorXd& mean,
	double mean_orientation,
	const Eigen::MatrixXd& cov_front,
	const Eigen::MatrixXd& cov_rear,
	bool unify_cov_scale = false
);

/**
 * @brief Computes a value of Gaussian described with mean vector and covariance matrix
 *
 * This is the most handy for multivariate Gaussians
 * Based on http://blog.sarantop.com/notes/mvn
 *
 * @tparam Tvec Eigen::VectorXd or social_nav_utils::Vector2d
 * @tparam Tmat Eigen::MatrixXd or social_nav_utils::Matrix2d
 * @param x vector to compute Gaussian for
 * @param mean vector of mean values
 * @param cov covariance matrix
 * @param n dimensionality of the problem
 * @return double Value of a Gaussian
 */
template <typename Tvec, typename Tmat>
double calculateGaussian(const Tvec& x, const Tvec& mean, const Tmat& cov, double n) {
	double sqrt2pi = std::sqrt(2 * M_PI);
	double quadform  = (x - mean).transpose() * cov.inverse() * (x - mean);
	double norm = std::pow(sqrt2pi, -n) * std::pow(cov.determinant(), -0.5);
	return norm * exp(-0.5 * quadform);
}

/**
 * @brief Computes a value of asymmetrical Gaussian described with a mean (at least 2 elem.) and 2 covariance matrices
 *
 * Evaluates whether x is located in front of the mean and selects appropriate covariance matrix to compute PDF
 *
 * @tparam Tvec Eigen::VectorXd or social_nav_utils::Vector2d
 * @tparam Tmat Eigen::MatrixXd or social_nav_utils::Matrix2d
 * @param x vector for whom the PDF is computed for
 * @param mean mean of the Gaussian
 * @param mean_orientation 'orientation' of the mean of the Gaussian (assuming that x does not contain this info)
 * @param cov_front covariance matrix for the front case
 * @param cov_rear covariance matrix for the rear case
 * @param unify_cov_scale adjusts scale of the output to avoid a step when front/rear covariances strongly differ
 * @return double
 */
template <typename Tvec, typename Tmat>
double calculateGaussianAsymmetrical(
	const Tvec& x,
	const Tvec& mean,
	double mean_orientation,
	const Tmat& cov_front,
	const Tmat& cov_rear,
	bool unify_cov_scale = false
) {
	// select covariance according to geometrical arrangement of x and mean
	RelativeLocation rel_loc(mean(0), mean(1), mean_orientation, x(0), x(1));
	Tmat cov;
	if (rel_loc.isFront()) {
		cov = cov_front;
	} else {
		cov = cov_rear;
	}

	double scale = 1.0;
	// unify to the scale in both directions, use the side with a smaller variance
	if (unify_cov_scale) {
		double maxfront = calculateGaussian(mean, mean, cov_front);
		double maxrear = calculateGaussian(mean, mean, cov_rear);
		// scale will affect only distribution with bigger variance
		double maxcurr = rel_loc.isFront() ? maxfront : maxrear;
		scale = std::max(maxrear, maxfront) / maxcurr;
	}

	return scale * calculateGaussian(x, mean, cov);
}

} // namespace social_nav_utils
