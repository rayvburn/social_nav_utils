#include <social_nav_utils/heading_direction_disturbance.h>

#include <social_nav_utils/gaussians.h>
#include <social_nav_utils/lines_intersection.h>
#include <social_nav_utils/relative_location.h>

#include <math.h>

namespace social_nav_utils {

HeadingDirectionDisturbance::HeadingDirectionDisturbance(
	double x_ego,
	double y_ego,
	double yaw_ego,
	double cov_xx_ego,
	double cov_xy_ego,
	double cov_yy_ego,
	double x_other,
	double y_other,
	double yaw_other,
	double vx_other,
	double vy_other,
	double occupancy_model_radius,
	double fov_ego
):
	pose_ego_(x_ego, y_ego, yaw_ego),
	cov_pos_ego_(cov_xx_ego, cov_xy_ego, cov_xy_ego, cov_yy_ego),
	pose_other_(x_other, y_other, yaw_other),
	vel_other_(vx_other, vy_other),
	ego_occupancy_model_radius_(occupancy_model_radius),
	fov_ego_(fov_ego)
{
	direction_disturbance_scale_ = computeDirectionDisturbance(
		pose_ego_(0),
		pose_ego_(1),
		pose_ego_(2),
		cov_pos_ego_(0, 0),
		cov_pos_ego_(0, 1),
		cov_pos_ego_(1, 1),
		pose_other_(0),
		pose_other_(1),
		pose_other_(2),
		ego_occupancy_model_radius_
	);
	RelativeLocation rel_loc(pose_ego_(0), pose_ego_(1), yaw_ego, pose_other_(0), pose_other_(1));
	fov_scale_ = computeFovScale(rel_loc.getAngle(), fov_ego_);
	speed_scale_ = computeSpeedScale(vel_other_(0), vel_other_(1));
	distance_scale_ = computeDistScale(pose_ego_(0), pose_ego_(1), pose_other_(0), pose_other_(1));
}

void HeadingDirectionDisturbance::normalize(double other_circumradius, double max_speed) {
	// let's assume that yaw of 'other' that  points straight into the center of 'ego'
	auto v_eo = pose_ego_ - pose_other_;
	double yaw_other_max_disturbance = std::atan2(v_eo(1), v_eo(0));
	double direction_disturbance_scale_max = computeDirectionDisturbance(
		pose_ego_(0),
		pose_ego_(1),
		pose_ego_(2),
		cov_pos_ego_(0, 0),
		cov_pos_ego_(0, 1),
		cov_pos_ego_(1, 1),
		pose_other_(0),
		pose_other_(1),
		yaw_other_max_disturbance,
		ego_occupancy_model_radius_
	);
	// let's assume that 'other' is located along the sight axis of the 'ego'
	double fov_scale_max = computeFovScale(0.0, fov_ego_);
	// simplified case (length of the velocity vector is not calculated here)
	double speed_scale_max = max_speed;
	// inverse proportional - minimum possible distance between 'other' and 'ego' centers
	double dist_scale_min = other_circumradius + ego_occupancy_model_radius_;

	// normalize scales
	direction_disturbance_scale_ /= direction_disturbance_scale_max;
	fov_scale_ /= fov_scale_max;
	speed_scale_ /= max_speed;
	distance_scale_ /= dist_scale_min;
}

double HeadingDirectionDisturbance::computeDirectionDisturbance(
	double x_ego,
	double y_ego,
	double yaw_ego,
	double cov_xx_ego,
	double cov_xy_ego,
	double cov_yy_ego,
	double x_other,
	double y_other,
	double yaw_other,
	double occupancy_model_radius
) {
	// make vectors really long so the intersection is appropriately detected
	Vector2d v_dir(VECTORS_LEN_INTERSECTION, 0.0);
	Rotation2Dd rot_ego(yaw_ego);
	Rotation2Dd rot_other(yaw_other);
	// vectors for shifting from the mean positions
	auto v_intsec_ego = rot_ego * v_dir;
	auto v_intsec_other = rot_other * v_dir;
	// find shifted positions from prolonged vectors
	Vector2d pos_ego(x_ego, y_ego);
	Vector2d pos_other(x_other, y_other);
	// compute points that are used to find intersection
	auto pos_ego_shifted1 = pos_ego - v_intsec_ego;
	auto pos_ego_shifted2 = pos_ego + v_intsec_ego;
	auto pos_other_shifted1 = pos_other - v_intsec_other;
	auto pos_other_shifted2 = pos_other + v_intsec_other;

	LinesIntersection lin_intsec(
		std::vector<double>{pos_ego_shifted1(0), pos_ego_shifted2(0)},
		std::vector<double>{pos_ego_shifted1(1), pos_ego_shifted2(1)},
		std::vector<double>{pos_other_shifted1(0), pos_other_shifted2(0)},
		std::vector<double>{pos_other_shifted1(1), pos_other_shifted2(1)}
	);

	// check if results are valid
	if (std::isnan(lin_intsec.getX()) || std::isnan(lin_intsec.getY())) {
		// direction axes are most likely parallel to each other (ego's vs other's)
		return 0.0;
	}

	// find covariance matrix of the occupancy model
	// 2-sigma rule
	auto var_occup_model = std::pow(occupancy_model_radius / SIGMA_RULE_NUM, 2.0);
	Matrix2d cov_occup(
		var_occup_model, 0.0,
		0.0, var_occup_model
	);

	// covariance matrix of the human position estimation uncertainty
	Matrix2d cov_pos_uncert(
		cov_xx_ego, cov_xy_ego,
		cov_xy_ego, cov_yy_ego
	);

	// resultant covariance
	Matrix2d cov_result = cov_occup + cov_pos_uncert;

	// find Gaussian at the intersection point
	Vector2d pos_intsec(lin_intsec.getX(), lin_intsec.getY());
	return calculateGaussian(pos_intsec, pos_ego, cov_result);
}

double HeadingDirectionDisturbance::computeFovScale(double relative_location_angle, double fov_ego) {
	// check whether the robot is located within person's FOV (only then affects human's behaviour);
	// 2 sigma rule is used here -> 2 sigma rule applied to the half of the FOV
	double fov_stddev = (fov_ego / 2.0) / SIGMA_RULE_NUM;
	double variance_fov = std::pow(fov_stddev, 2);
	// starting from the left side, half of the `fov_ego` is located in 0.0 and rel_loc is 0.0
	// when obstacle is in front of the object
	return calculateGaussian(relative_location_angle, 0.0, variance_fov);
}

double HeadingDirectionDisturbance::computeSpeedScale(double vel_x_other, double vel_y_other) {
	// check if robot faces person but only rotates or is moving fast
	return std::hypot(vel_x_other, vel_y_other);
}

double HeadingDirectionDisturbance::computeDistScale(double x_ego, double y_ego, double x_other, double y_other) {
	// check how far the robot is from the person (euclidean distance)
	return std::hypot(x_other - x_ego, y_other - y_ego);
}

} // namespace social_nav_utils
