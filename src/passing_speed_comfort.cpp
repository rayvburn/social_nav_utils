#include <social_nav_utils/passing_speed_comfort.h>

#include <math.h>
#include <functional>

namespace social_nav_utils {

PassingSpeedComfort::PassingSpeedComfort(double distance, double robot_speed) {
	comfort_ = computeSpeedComfort(distance, robot_speed);
}

double PassingSpeedComfort::computeSpeedComfort(double distance, double speed) {
	/*
	 * Authors of "The effect of robot speed on comfortable passing distances" did not established a full model
	 * representing comfort as a function of speed and distance for the passing scenario. Thus, an approximation
	 * of their results was designed.
	 *
	 * Namely, 2 exponential models for closer and further passing distances were defined based on their results (Fig. 7)
	 */
	// general fitting model lambda function
	auto model_exp2_fun = [](double a, double b, double c, double d, double x) -> double {
		// Based on Matlab's `exp2` model
		return a * std::exp(b * x) + c * std::exp(d * x);
	};
	// fitted model for close passing distances
	auto model_close_fun = [model_exp2_fun](double speed) -> double {
		// coefficients
		const double a = 6.023;
		const double b = -0.2822;
		const double c = -0.9639;
		const double d = -3.905;
		return model_exp2_fun(a, b, c, d, speed);
	};
	// fitted model for far passing distances
	auto model_far_fun = [model_exp2_fun](double speed) -> double {
		// coefficients
		const double a = 8.385;
		const double b = -0.2633;
		const double c = -3.759;
		const double d = -2.304;
		return model_exp2_fun(a, b, c, d, speed);
	};

	// upper boundary
	const double CLOSE_DIST_THRESHOLD = 0.6;
	// lower boundary
	const double FAR_DIST_THRESHOLD = 0.8;

	if (distance <= CLOSE_DIST_THRESHOLD) {
		return model_close_fun(speed);
	} else if (distance >= CLOSE_DIST_THRESHOLD) {
		return model_far_fun(speed);
	}

	// mixture of models for distances between CLOSE_DIST_THRESHOLD and FAR_DIST_THRESHOLD
	// results is average of both outputs
	double comfort_close = model_close_fun(speed);
	double comfort_far = model_far_fun(speed);

	double mixing_range = FAR_DIST_THRESHOLD - CLOSE_DIST_THRESHOLD;
	double dist_from_far = FAR_DIST_THRESHOLD - distance;
	double close_factor = dist_from_far / mixing_range;
	double far_factor = 1.0 - close_factor;

	return close_factor * comfort_close + far_factor * comfort_far;
}

} // namespace social_nav_utils
