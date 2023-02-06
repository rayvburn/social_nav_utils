#include <social_nav_utils/heading_direction_disturbance.h>

#include <social_nav_utils/gaussians.h>

#include <angles/angles.h>
#include <stdexcept>
#include <math.h>
#include <string>
#include <tuple>

namespace social_nav_utils {

HeadingDirectionDisturbance::HeadingDirectionDisturbance(
	double x_ego,
	double y_ego,
	double yaw_ego,
	double fov_ego,
	double x_other,
	double y_other,
	double yaw_other,
	double vx_other,
	double vy_other
) {
	double relative_location_angle = 0.0;
	std::tie(direction_disturbance_, relative_location_angle) = computeDirectionDisturbance(
		x_ego,
		y_ego,
		yaw_ego,
		x_other,
		y_other,
		yaw_other
	);
	fov_scale_ = computeFovScale(relative_location_angle, fov_ego);
	speed_scale_ = computeSpeedScale(vx_other, vy_other);
	distance_scale_ = computeDistScale(x_ego, y_ego, x_other, y_other);
}

std::pair<double, double> HeadingDirectionDisturbance::computeDirectionDisturbance(
	double x_ego,
	double y_ego,
	double yaw_ego,
	double x_other,
	double y_other,
	double yaw_other
) {
	double dist_vector[2] = {0.0};
	dist_vector[0] = x_other - x_ego;
	dist_vector[1] = y_other - y_ego;

	// length of the vector
	double dist_vector_length = std::sqrt(
	  std::pow(dist_vector[0], 2)
	  + std::pow(dist_vector[1], 2)
	);

	// direction of vector connecting robot and person (defines where the robot is located in relation to a person [ego agent])
	double dist_vector_angle = std::atan2(dist_vector[1], dist_vector[0]);

	// relative location vector angle (defines side where the robot is located in relation to a person)
	double rel_loc_angle = angles::normalize_angle(dist_vector_angle - yaw_ego);

	// old notation: alpha-beta can be mapped to: i -> robot, j -> person
	double gamma = angles::normalize_angle(rel_loc_angle - yaw_other);

	// calculate threshold angle values, normalize angles
	/// indicates that j moves in the same direction as i
	double gamma_eq = angles::normalize_angle(dist_vector_angle - 2 * yaw_ego);
	/// indicates that j moves in a direction opposite to i
	double gamma_cc = angles::normalize_angle(M_PI - 2 * yaw_ego);
	/// indicates that a ray created from a centre point and a heading of j crosses the centre point of i
	double gamma_opp = angles::normalize_angle(gamma_eq - M_PI);

	/*
	 * Find range of angles that indicate <opposite, crossing in front, etc> motion direction of the robot towards person
	 * e.g. opposite direction adjoins with `cross behind` and `outwards` ranges.
	 * Range between direction regions can be used as a variance to model gaussian cost
	 */
	// decode relative location (right/left side)
	std::string relative_location_side = "unknown";
	if (rel_loc_angle < 0.0) {
	  relative_location_side = "right";
	} else if (rel_loc_angle >= 0.0) {
	  relative_location_side = "left";
	}

	// not all angles are required in this method, some values are computed for future use
	double gamma_cf_start = 0.0;
	double gamma_cf_finish = 0.0;
	double gamma_cb_start = 0.0;
	double gamma_cb_finish = 0.0;
	double gamma_out_start = 0.0;
	double gamma_out_finish = 0.0;

	if (relative_location_side == "right") {
		gamma_cf_start = gamma_cc;
		gamma_cf_finish = gamma_eq;
		gamma_cb_start = gamma_opp;
		gamma_cb_finish = gamma_cc;
		gamma_out_start = gamma_eq;
		gamma_out_finish = gamma_opp;
	} else if (relative_location_side == "left") {
		gamma_cf_start = gamma_eq;
		gamma_cf_finish = gamma_cc;
		gamma_cb_start = gamma_cc;
		gamma_cb_finish = gamma_opp;
		gamma_out_start = gamma_opp;
		gamma_out_finish = gamma_eq;
	} else {
		throw std::runtime_error("Unknown value of relative location");
	}

	double gamma_cf_range = std::abs(angles::shortest_angular_distance(gamma_cf_start, gamma_cf_finish));
	double gamma_cb_range = std::abs(angles::shortest_angular_distance(gamma_cb_start, gamma_cb_finish));
	double gamma_out_range = std::abs(angles::shortest_angular_distance(gamma_out_start, gamma_out_finish));

	// determine, how wide the region of, cross_center direction angles, will be (assuming circular model of the person)
	const double PERSON_MODEL_RADIUS = 0.4;
	// we must keep arcsin argument below 1.0, otherwise NAN will be returned instead of a very big angle
	double dist_gamma_range = std::max(PERSON_MODEL_RADIUS, dist_vector_length);
	double gamma_cc_range = 2.0 * std::asin(PERSON_MODEL_RADIUS / dist_gamma_range);
	// Variance is computed according 68–95–99.7 rule https://en.wikipedia.org/wiki/68%E2%80%9395%E2%80%9399.7_rule
	double gamma_cc_stddev = (gamma_cc_range / 2.0) / 3.0;
	double gamma_cc_variance = std::pow(gamma_cc_stddev, 2);

	/*
	 * Note that we assume that gaussian cost of disturbance exists only within bounds of following direction angles:
	 * - crossing-center (i.e. opposite and moving towards the person center)
	 * - crossing-in-front
	 */
	// 1D Gaussian function, note that angle domain wraps at 3.14 so we must check for maximum of gaussians
	// located at gamma_X and shifted 2 * pi to the left and right; gamma angle should already be normalized here
	double gaussian_dir_cc = social_nav_utils::calculateGaussianAngle(gamma, gamma_cc, gamma_cc_variance);

	// 3 sigma rule - let the cost spread only over the CF region
	double gamma_cf_stddev = (gamma_cf_range / 2.0) / 3.0;
	double gamma_cf_variance = std::pow(gamma_cf_stddev, 2);
	// mean - center of the cross front region
	double gamma_cf_center = angles::normalize_angle(gamma_cf_start + gamma_cf_range / 2.0);
	double gaussian_dir_cf = social_nav_utils::calculateGaussianAngle(gamma, gamma_cf_center, gamma_cf_variance);

	double gaussian_dir_result = std::max(gaussian_dir_cc, gaussian_dir_cf);

	return std::make_pair(gaussian_dir_result, rel_loc_angle);
}

double HeadingDirectionDisturbance::computeFovScale(double relative_location_angle, double fov_ego) {
	// check whether the robot is located within person's FOV (only then affects human's behaviour);
	// again, 3 sigma rule is used here -> 3 sigma rule applied to the half of the FOV
	double fov_stddev = (fov_ego / 2.0) / 3.0;
	double variance_fov = std::pow(fov_stddev, 2);
	// starting from the left side, half of the `fov_ego` is located in 0.0 and rel_loc is 0.0
	// when obstacle is in front of the object
	double gaussian_fov = social_nav_utils::calculateGaussian(relative_location_angle, 0.0, variance_fov);
	return gaussian_fov;
}

double HeadingDirectionDisturbance::computeSpeedScale(double vel_x_other, double vel_y_other) {
	// check if robot faces person but only rotates or is moving fast
	double speed_factor = std::sqrt(
		std::pow(vel_x_other, 2)
		+ std::pow(vel_y_other, 2)
	);
	return speed_factor;
}

double HeadingDirectionDisturbance::computeDistScale(double x_ego, double y_ego, double x_other, double y_other) {
	// check how far the robot is from the person
	double eucl_dist = std::sqrt(
		std::pow(x_other - x_ego, 2)
		+ std::pow(y_other - y_ego, 2)
	);
	// exponent provides approx 0.5 @ 1 m between centers of robot and person
	static constexpr auto DIST_FACTOR_EXP = -0.8;
	double dist_factor = std::exp(DIST_FACTOR_EXP * eucl_dist);
}

} // namespace social_nav_utils
