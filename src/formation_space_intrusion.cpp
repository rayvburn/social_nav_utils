#include <social_nav_utils/formation_space_intrusion.h>

#include <social_nav_utils/ellipse_fitting.h>
#include <social_nav_utils/gaussians.h>

#include <math.h>

// may prevent compilation errors occurring at calls to matrix.inverse()
#include <eigen3/Eigen/LU>

namespace social_nav_utils {

FormationSpaceIntrusion::FormationSpaceIntrusion(
	double ospace_pos_x,
	double ospace_pos_y,
	double ospace_orientation,
	double ospace_variance_x,
	double ospace_variance_y,
	double pos_center_variance_xx,
	double pos_center_variance_xyyx,
	double pos_center_variance_yy,
	double robot_pos_x,
	double robot_pos_y
):
	intrusion_scale_(NAN),
	ospace_pos_x_(ospace_pos_x),
	ospace_pos_y_(ospace_pos_y),
	ospace_orientation_(ospace_orientation),
	ospace_variance_x_(ospace_variance_x),
	ospace_variance_y_(ospace_variance_y),
	pos_center_variance_xx_(pos_center_variance_xx),
	pos_center_variance_xyyx_(pos_center_variance_xyyx),
	pos_center_variance_yy_(pos_center_variance_yy),
	robot_pos_x_(robot_pos_x),
	robot_pos_y_(robot_pos_y)
{
	intrusion_scale_ = computeFormationSpaceGaussian(
		ospace_pos_x_,
		ospace_pos_y_,
		ospace_orientation_,
		ospace_variance_x_,
		ospace_variance_y_,
		pos_center_variance_xx_,
		pos_center_variance_xyyx_,
		pos_center_variance_yy_,
		robot_pos_x_,
		robot_pos_y_
	);
}

void FormationSpaceIntrusion::normalize() {
	// find max of Gaussian knowing the current arrangement and certainty - compute gaussian at mean position
	double intrusion_max = computeFormationSpaceGaussian(
		ospace_pos_x_,
		ospace_pos_y_,
		ospace_orientation_,
		ospace_variance_x_,
		ospace_variance_y_,
		pos_center_variance_xx_,
		pos_center_variance_xyyx_,
		pos_center_variance_yy_,
		ospace_pos_x_,
		ospace_pos_y_
	);
	intrusion_scale_ /= intrusion_max;
}

double FormationSpaceIntrusion::computeFormationSpaceGaussian(
	double ospace_pos_x,
	double ospace_pos_y,
	double ospace_orientation,
	double ospace_variance_x,
	double ospace_variance_y,
	double pos_center_variance_xx,
	double pos_center_variance_xyyx,
	double pos_center_variance_yy,
	double robot_pos_x,
	double robot_pos_y
) {
	// create matrix for covariance rotation
	Eigen::MatrixXd rot(2, 2);
	rot << std::cos(ospace_orientation), -std::sin(ospace_orientation),
		std::sin(ospace_orientation), std::cos(ospace_orientation);

	// create covariance matrix of the personal zone model
	Eigen::MatrixXd cov_fsi_init(2, 2);
	cov_fsi_init << ospace_variance_x, 0.0, 0.0, ospace_variance_y;

	// rotate covariance matrix
	Eigen::MatrixXd cov_fsi(2, 2);
	cov_fsi = rot * cov_fsi_init * rot.inverse();

	// create covariance matrix of the position estimation uncertainty
	Eigen::MatrixXd cov_pos(2, 2);
	cov_pos << pos_center_variance_xx, pos_center_variance_xyyx, pos_center_variance_xyyx, pos_center_variance_yy;

	// resultant covariance matrices (variances summed up)
	Eigen::MatrixXd cov_result(2, 2);
	cov_result = cov_pos + cov_fsi;

	// prepare vectors for gaussian calculation
	// position to check Gaussian against - position of robot
	Eigen::VectorXd x_pos(2);
	x_pos << robot_pos_x, robot_pos_y;
	// mean - position of the group
	Eigen::VectorXd mean_pos(2);
	mean_pos << ospace_pos_x, ospace_pos_y;

	return calculateGaussian(x_pos, mean_pos, cov_result);
}

} // namespace social_nav_utils
