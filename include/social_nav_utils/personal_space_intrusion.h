#pragma once

namespace social_nav_utils {

class PersonalSpaceIntrusion {
public:
	/**
	 * @brief Constructor of personal space intrusion cost function
	 *
	 * For parameters description, refer to the @ref computePersonalSpaceGaussian
	 */
	PersonalSpaceIntrusion(
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
		bool unify_asymmetry_scale = false
	);

	/**
	 * Normalizes results to the worst case for the current arrangement
	 *
	 * Refers Gaussian at robot location to the Gaussian at human position
	 * After call to this method scale available by getter method will be normalized.
	 */
	void normalize();

	/// Returns scale of the intrusion into personal space according to the arrangement defined by ctor arguments
	double getScale() const {
		return intrusion_scale_;
	}

	/**
	 * @brief Computes value of a Gaussian modelling the personal space
	 *
	 * Includes pose uncertainty of the human
	 *
	 * @param person_pos_x
	 * @param person_pos_y
	 * @param person_orient_yaw
	 * @param person_pos_cov_xx
	 * @param person_pos_cov_xy
	 * @param person_pos_cov_yx
	 * @param person_pos_cov_yy
	 * @param person_ps_var_front variance along the front direction of a person
	 * @param person_ps_var_rear variance along the rear direction of a person
	 * @param person_ps_var_side variance along the direction of person side
	 * @param robot_pos_x
	 * @param robot_pos_y
	 * @param unify_asymmetry_scale adjusts scale of the output to avoid a step when front/rear covariances strongly differ
	 * @return double
	 */
	static double computePersonalSpaceGaussian(
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
		bool unify_asymmetry_scale = false
	);

protected:
	double intrusion_scale_;

	double person_pos_x_;
	double person_pos_y_;
	double person_orient_yaw_;
	double person_pos_cov_xx_;
	double person_pos_cov_xy_;
	double person_pos_cov_yx_;
	double person_pos_cov_yy_;
	double person_ps_var_front_;
	double person_ps_var_rear_;
	double person_ps_var_side_;
	double robot_pos_x_;
	double robot_pos_y_;
	bool unify_asymmetry_scale_;
};

} // namespace social_nav_utils
