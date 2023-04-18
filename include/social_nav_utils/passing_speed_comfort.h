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
	 * @param distance distance between centers of the robot and the human
	 * @param robot_speed
	 */
	PassingSpeedComfort(double distance, double robot_speed);

	double getComfort() const {
		return comfort_;
	}

	/**
	 * This method bases on results from:
	 * Neggers et al. "The effect of robot speed on comfortable passing distances" (2022)
	 */
	static double computeSpeedComfort(double distance, double robot_speed);

protected:
	double comfort_;
};

} // namespace social_nav_utils
