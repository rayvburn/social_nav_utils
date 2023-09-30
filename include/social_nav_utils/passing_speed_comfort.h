#pragma once

namespace social_nav_utils {

/**
 * Approximates a value of the human comfort when he is passed by the robot at a specific distance with a certain speed
 *
 * @note (internal) matlab script for fitting is named `passing_speed_cost_function_fitting.m`
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
	 * @param distance_max
	 * @param speed_min
	 */
	PassingSpeedComfort(
		double distance,
		double robot_speed,
		double distance_min = 0.275,
		double speed_max = 0.55,
		double distance_max = 5.0,
		double speed_min = 0.1
	);

	double getComfort() const {
		return comfort_;
	}

	double getComfortNormalized() const {
		return comfort_normalized_;
	}

	/**
	 * This method bases on results from:
	 * Neggers et al. "The effect of robot speed on comfortable passing distances" (2022)
	 */
	static double computeSpeedComfort(double distance, double robot_speed);

protected:
	double comfort_;
	/// @ref comfort_ normalized according to the highest and smallest values recorded
	double comfort_normalized_;
};

} // namespace social_nav_utils
