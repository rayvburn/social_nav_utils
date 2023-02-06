#pragma once

#include <utility>

namespace social_nav_utils {

/**
 * @brief Calculates value of 'ego' agent disturbance induced by 'other' agent motion (its direction in particular).
 *
 * Typically, in social robotics applications, 'ego' will be the human and 'other' will be the robot, whose motion
 * is typically evaluated with this metric.
 *
 * It is assumed that 'ego' and 'other' agents are nearby.
 *
 * Disturbance is modelled by a Gaussian function. Its values are computed by arguments given in domain of angles.
 * For further details check `dirCross` concept (location of the intersection point of i and j direction rays
 * in relation to the i centre) in `hubero_local_planner`. Here, `i` is the person and `j` is the robot.
 */
class HeadingDirectionDisturbance {
public:
	/**
	 * @brief Parameterized constructor
	 * @param x_ego
	 * @param y_ego
	 * @param yaw_ego
	 * @param fov_ego total angular field of view of the ego agent
	 * @param x_other
	 * @param y_other
	 * @param yaw_other
	 * @param vx_other
	 * @param vy_other
	 */
	HeadingDirectionDisturbance(
		double x_ego,
		double y_ego,
		double yaw_ego,
		double fov_ego,
		double x_other,
		double y_other,
		double yaw_other,
		double vx_other,
		double vy_other
	);

	double getDirectionScale() const {
		return direction_disturbance_;
	}
	double getFovScale() const {
		return fov_scale_;
	}
	double getSpeedScale() const {
		return speed_scale_;
	}
	double getDistScale() const {
		return distance_scale_;
	}
	double getScale() const {
		return direction_disturbance_ * fov_scale_ * speed_scale_ * distance_scale_;
	}

	static std::pair<double, double> computeDirectionDisturbance(
		double x_ego,
		double y_ego,
		double yaw_ego,
		double x_other,
		double y_other,
		double yaw_other
	);

	static double computeFovScale(double relative_location_angle, double fov_ego);

	static double computeSpeedScale(double vel_x_other, double vel_y_other);

	static double computeDistScale(double x_ego, double y_ego, double x_other, double y_other);

protected:
	double direction_disturbance_;
	double fov_scale_;
	double speed_scale_;
	double distance_scale_;
};

} // namespace social_nav_utils
