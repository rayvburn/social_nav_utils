#pragma once

namespace social_nav_utils {

class FormationSpaceIntrusion {
public:
	/**
	 * @brief Constructor of F-formation's O-space intrusion cost function
	 *
	 * For parameters description, refer to the @ref computeFormationSpaceGaussian
	 */
	FormationSpaceIntrusion(
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
	);

	/**
	 * Normalizes results to the worst case for the current arrangement
	 *
	 * Refers Gaussian at robot location to the Gaussian at O-space center
	 * After call to this method scale available by getter method will be normalized.
	 */
	void normalize();

	/// Returns scale of the intrusion into F-formation's O-space space according to the arrangement defined by ctor arguments
	double getScale() const {
		return intrusion_scale_;
	}

	/**
	 * @brief Computes value of a Gaussian (given by method parameters) at given position
	 *
	 * @param ospace_pos_x
	 * @param ospace_pos_y
	 * @param ospace_orientation
	 * @param ospace_variance_x
	 * @param ospace_variance_y
	 * @param pos_center_variance_xx
	 * @param pos_center_variance_xyyx
	 * @param pos_center_variance_yy
	 * @param robot_pos_x
	 * @param robot_pos_y
	 * @return double
	 *
	 * @sa computeFormationSpaceGaussian
	 */
	static double computeFormationSpaceGaussian(
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
	);

protected:
	double intrusion_scale_;

	double ospace_pos_x_;
	double ospace_pos_y_;
	double ospace_orientation_;
	double ospace_variance_x_;
	double ospace_variance_y_;
	double pos_center_variance_xx_;
	double pos_center_variance_xyyx_;
	double pos_center_variance_yy_;
	double robot_pos_x_;
	double robot_pos_y_;
};

} // namespace social_nav_utils
