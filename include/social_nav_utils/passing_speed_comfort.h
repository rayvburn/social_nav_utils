#pragma once

#include <functional>
#include <vector>

namespace social_nav_utils {

/**
 * Estimates a value of the human comfort when he/she is passed by the robot.
 *
 * A specific distance and a certain speed are considered to estimate the comfort.
 *
 * The results from Neggers et al. "The effect of robot speed on comfortable passing distances" (2022) were used
 * to model the human comfort. The model is based on the distance between the centers of a human and a robot
 * and the speed of the robot.
 *
 * Two methods were implemented:
 * - simple - two separate curves were fitted to the source data (Fig. 7 of the referenced paper). The first model
 *   is prepared for distances between 0.0 m and 0.6 m between centers of the human and the robot). The second model
 *   is prepared for distances bigger than 0.8 m between centers. For the 0.6 - 0.8 range, the mixture of models
 *   is computed. Implemented in the @ref computeSpeedComfort
 * - a spline fitting - the source data (Fig. 7 of the referenced paper) were fitted to a single model describing
 *   the human comfort phenomenon. Implemented in the @ref computeSpeedComfortSpline
 * @note (internal) matlab script for fitting the simple form is named `passing_speed_cost_function_fitting.m`
 * @note (internal) matlab script for fitting is named `passing_speed_cost_function_fitting_single_model.m`
 */
class PassingSpeedComfort {
public:
	/**
	 * @brief Constructor
	 *
	 * The worst case scenario (minimum comfort), i.e., minimum distance between the human and a robot and the maximum
	 * speed of the robot, is used for the normalization of the comfort value.
	 *
	 * On the other hand, the best case scenario, i.e., maximum distance and the minimum speed is used to compute
	 * the maximum possible comfort value (which is used for the normalization of the comfort value).
	 *
	 * @param distance current distance between centers of the robot and the human
	 * @param robot_speed
	 * @param distance_min the minimum possible distance between the human and a robot (typically the circumradius
	 * of the robot's footprint)
	 * @param speed_max the maximum possible speed achievable by the robot
	 * @param distance_max distance at which the maximum comfort value occurs (at the speed of @ref speed_min);
	 * for the simple method: 0.808080808080808 and the spline fitting: 1.40
	 * @param speed_min speed at which the maximum comfort value occurs (at the distance of @ref distance_max);
	 * for the simple method: 0.424242424242424 and the spline fitting: 0.00
	 * @param simple_method Whether to use the simple method (true, default) of the spline fitting approach (false)
	 */
	PassingSpeedComfort(
		double distance,
		double robot_speed,
		double distance_min = 0.275,
		double speed_max = 0.55,
		double distance_max = 1.40,
		double speed_min = 0.0,
		bool simple_method = false
	);

	double getComfort() const {
		return comfort_;
	}

	double getComfortNormalized() const {
		return comfort_normalized_;
	}

	double getDiscomfortNormalized() const {
		return std::max(std::min(1.0 - getComfortNormalized(), 1.0), 0.0);
	}

	/**
	 * Estimates the normalized (0.0 - 1.0) human comfort using the simple method
	 */
	static double computeSpeedComfort(double distance, double robot_speed);

	/**
	 * Estimates the normalized (0.0 - 1.0) human comfort using the simple method
	 */
	static double computeSpeedComfortNormalized(
		double dist,
		double speed,
		double dist_min,
		double speed_max,
		double dist_max,
		double speed_min,
		std::function<double(double,double)>& calc_fun
	);

	/**
	 * Estimates the normalized (0.0 - 1.0) human comfort using the simple method
	 */
	static double computeSpeedComfortNormalized(
		double comfort,
		double dist_min,
		double speed_max,
		double dist_max,
		double speed_min,
		std::function<double(double,double)>& calc_fun
	);

	/**
	 * Computes the human comfort using the spline fitting approach
	 */
	static double computeSpeedComfortSpline(double distance, double robot_speed);

	/**
	 * Estimates the normalized (0.0 - 1.0) human comfort using the spline fitting approach
	 */
	static double computeSpeedComfortSplineNormalized(
		double dist,
		double speed,
		double dist_min,
		double speed_max,
		double dist_max,
		double speed_min,
		std::function<double(double,double)>& spline_fun
	);

	/**
	 * Estimates the normalized (0.0 - 1.0) human comfort using the spline fitting approach
	 */
	static double computeSpeedComfortSplineNormalized(
		double comfort,
		double dist_min,
		double speed_max,
		double dist_max,
		double speed_min,
		std::function<double(double,double)>& spline_fun
	);

	/**
	 * Estimates the normalized (0.0 - 1.0) human comfort using the spline fitting approach
	 */
	static double computeSpeedComfortSplineNormalized(
		double comfort,
		double comfort_min,
		double comfort_max
	);

	/**
	 * @brief Computes a value of a spline defined in a ppform
	 *
	 * Spline form reference: https://www.mathworks.com/help/curvefit/the-ppform.html
	 *
	 * Calculation method recreated using the following instruction:
	 * https://www.mathworks.com/matlabcentral/answers/77100-reconstruct-multivariate-spline-from-csapi
	 *
	 * @param x a coordinate to compute the spline model for
	 * @param y a coordinate to compute the spline model for
	 * @param coefficients
	 * @param breaks_x
	 * @param breaks_y
	 * @param order_x
	 * @param order_y
	 * @return double
	 */
	static double computeSplineBivariate(
		double x,
		double y,
		const std::vector<std::vector<std::vector<std::vector<double>>>>& coefficients,
		const std::vector<double>& breaks_x,
		const std::vector<double>& breaks_y,
		int order_x,
		int order_y
	);

protected:
	double comfort_;
	/// @ref comfort_ normalized according to the highest and smallest values recorded
	double comfort_normalized_;
};

} // namespace social_nav_utils
