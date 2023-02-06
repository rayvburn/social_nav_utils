#pragma once

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
	double sigma_h,
	double sigma_r,
	double sigma_s
);

} // namespace social_nav_utils
