#include <social_nav_utils/personal_space_intrusion.h>

#include <social_nav_utils/distance_vector.h>
#include <social_nav_utils/relative_location.h>
#include <social_nav_utils/gaussians.h>

#include <social_nav_utils/math/core.h>

namespace social_nav_utils {

PersonalSpaceIntrusion::PersonalSpaceIntrusion(
	double person_pos_x,
	double person_pos_y,
	double person_orient_yaw,
	double person_pos_cov_xx,
	double person_pos_cov_xy,
	double person_pos_cov_yx,
	double person_pos_cov_yy,
	double person_ps_var_front,
	double person_ps_var_rear,
	double person_ps_var_side,
	double robot_pos_x,
	double robot_pos_y,
	bool unify_asymmetry_scale
):
	intrusion_scale_(NAN),
	person_pos_x_(person_pos_x),
	person_pos_y_(person_pos_y),
	person_orient_yaw_(person_orient_yaw),
	person_pos_cov_xx_(person_pos_cov_xx),
	person_pos_cov_xy_(person_pos_cov_xy),
	person_pos_cov_yx_(person_pos_cov_yx),
	person_pos_cov_yy_(person_pos_cov_yy),
	person_ps_var_front_(person_ps_var_front),
	person_ps_var_rear_(person_ps_var_rear),
	person_ps_var_side_(person_ps_var_side),
	robot_pos_x_(robot_pos_x),
	robot_pos_y_(robot_pos_y),
	unify_asymmetry_scale_(unify_asymmetry_scale)
{
	intrusion_scale_ = computePersonalSpaceGaussian(
		person_pos_x_,
		person_pos_y_,
		person_orient_yaw_,
		person_pos_cov_xx_,
		person_pos_cov_xy_,
		person_pos_cov_yx_,
		person_pos_cov_yy_,
		person_ps_var_front_,
		person_ps_var_rear_,
		person_ps_var_side_,
		robot_pos_x_,
		robot_pos_y_,
		unify_asymmetry_scale_
	);
}

void PersonalSpaceIntrusion::normalize() {
	// find max of Gaussian knowing the current arrangement and certainty - compute gaussian at mean position
	double intrusion_max = computePersonalSpaceGaussian(
		person_pos_x_,
		person_pos_y_,
		person_orient_yaw_,
		person_pos_cov_xx_,
		person_pos_cov_xy_,
		person_pos_cov_yx_,
		person_pos_cov_yy_,
		person_ps_var_front_,
		person_ps_var_rear_,
		person_ps_var_side_,
		person_pos_x_,
		person_pos_y_,
		unify_asymmetry_scale_
	);
	intrusion_scale_ /= intrusion_max;
}

double PersonalSpaceIntrusion::computePersonalSpaceGaussian(
	double person_pos_x,
	double person_pos_y,
	double person_orient_yaw,
	double person_pos_cov_xx,
	double person_pos_cov_xy,
	double person_pos_cov_yx,
	double person_pos_cov_yy,
	double person_ps_var_front,
	double person_ps_var_rear,
	double person_ps_var_side,
	double robot_pos_x,
	double robot_pos_y,
	bool unify_asymmetry_scale
) {
	// create matrix for covariance rotation
	double rot_angle = person_orient_yaw;
	Rotation2Dd rot(rot_angle);

	// create human position uncertainty matrix
	Matrix2d cov_p(
		person_pos_cov_xx, person_pos_cov_xy,
		person_pos_cov_yx, person_pos_cov_yy
	);

	// prepare vectors for gaussian calculation
	// position to check Gaussian against - position of robot
	Vector2d x_pos(robot_pos_x, robot_pos_y);
	// mean - position of human
	Vector2d mean_pos(person_pos_x, person_pos_y);

	/*
	* Perfect Gaussians in terms of mathematical description. Selecting `unify_asymmetry_scale`,
	* With asymmetry there will be a bump across the center axis due to different variances (thus maximums)
	*/
	if (!unify_asymmetry_scale) {
		// aka phi
		DistanceVector dist_vector(
			person_pos_x,
			person_pos_y,
			robot_pos_x,
			robot_pos_y
		);

		// aka delta
		RelativeLocation rel_loc(dist_vector, person_orient_yaw);

		// choose variance
		double var_h_heading = person_ps_var_rear;
		if (rel_loc.getAngle() <= M_PI_2) {
			var_h_heading = person_ps_var_front;
		}

		// create covariance matrix of the personal zone model
		Matrix2d cov_psi_init(var_h_heading, 0.0, 0.0, person_ps_var_side);

		// rotate covariance matrix
		Matrix2d cov_psi = rot * cov_psi_init * rot.inverse();

		// resultant covariance matrix
		Matrix2d cov_result = cov_p + cov_psi;

		// we already know the covariance so there is no need to evaluate the 'Asymmetrical' Gaussian case for `x_pos`
		return calculateGaussian(x_pos, mean_pos, cov_result);
	}

	/*
	* More popular version - the Gaussian with higher variance is prolonged in the uppper direction according
	* to the second one's maximum (in the mean pose)
	*/
	// create covariance matrices of the personal zone model
	Matrix2d cov_psi_init_front(person_ps_var_front, 0.0, 0.0, person_ps_var_side);
	Matrix2d cov_psi_init_rear(person_ps_var_rear, 0.0, 0.0, person_ps_var_side);

	// rotate covariance matrices
	Matrix2d cov_psi_front = rot * cov_psi_init_front * rot.inverse();
	Matrix2d cov_psi_rear = rot * cov_psi_init_rear * rot.inverse();

	// resultant covariance matrices (variances summed up)
	Matrix2d cov_result_front = cov_p + cov_psi_front;
	Matrix2d cov_result_rear = cov_p + cov_psi_rear;

	// compute value of asymmetric Gaussian
	return calculateGaussianAsymmetrical(
		x_pos,
		mean_pos,
		person_orient_yaw,
		cov_result_front,
		cov_result_rear,
		true
	);
}

} // namespace social_nav_utils
