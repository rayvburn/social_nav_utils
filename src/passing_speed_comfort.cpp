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
		const double a = 6.022541906829932;
		const double b = -0.282233847093237;
		const double c = -0.963915948883583;
		const double d = -3.904914546865386;
		return model_exp2_fun(a, b, c, d, speed);
	};
	// fitted model for far passing distances
	auto model_far_fun = [model_exp2_fun](double speed) -> double {
		// coefficients
		const double a = 6.518175639643890;
		const double b = 0.098435276370722;
		const double c = -0.427768878118209;
		const double d = 1.017851222030475;
		return model_exp2_fun(a, b, c, d, speed);
	};

	// upper boundary
	const double CLOSE_DIST_THRESHOLD = 0.6;
	// lower boundary
	const double FAR_DIST_THRESHOLD = 0.8;

	// store the comfort value computed for any of the cases below
	double comfort = 0.0;
	if (distance <= CLOSE_DIST_THRESHOLD) {
		comfort = model_close_fun(speed);
	} else if (distance >= FAR_DIST_THRESHOLD) {
		comfort = model_far_fun(speed);
	} else {
		// mixture of models for distances between CLOSE_DIST_THRESHOLD and FAR_DIST_THRESHOLD
		// results is the average of both outputs
		double comfort_close = model_close_fun(speed);
		double comfort_far = model_far_fun(speed);

		double mixing_range = FAR_DIST_THRESHOLD - CLOSE_DIST_THRESHOLD;
		double dist_from_far = FAR_DIST_THRESHOLD - distance;
		double close_factor = dist_from_far / mixing_range;
		double far_factor = 1.0 - close_factor;

		comfort = close_factor * comfort_close + far_factor * comfort_far;
	}
	// trim to the lower bound
	return std::max(comfort, 0.0);
}

} // namespace social_nav_utils
