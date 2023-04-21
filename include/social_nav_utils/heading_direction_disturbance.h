#pragma once

#include <social_nav_utils/math/core.h>

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
	/// Default width of the region of cross_center direction angles (assuming circular model of the 'ego' agent)
	static constexpr auto OCCUPANCY_MODEL_RADIUS_DEFAULT = 0.28;
	/// Default value of human's FOV, corresponds to 210 degrees
	static constexpr auto FOV_DEFAULT = 3.6652;
	/// Default circumradius of the robot
	static constexpr auto CIRCUMRADIUS_DEFAULT = 0.275;
	/// Default maximum linear velocity of the robot
	static constexpr auto MAX_SPEED_DEFAULT = 0.55;
	/// Length of vectors when looking for an intersection point. See usage for details.
	static constexpr auto VECTORS_LEN_INTERSECTION = 1000.0;
	/// Number of sigmas included in calculations, see https://en.wikipedia.org/wiki/68%E2%80%9395%E2%80%9399.7_rule
	static constexpr auto SIGMA_RULE_NUM = 2.0;

	/**
	 * @brief Constructor
	 *
	 * Ego: agent that is being disturbed by a the motion of 'other' agent. Typically, 'ego' is a human,
	 * whereas 'other' is a robot.
	 *
	 * @param x_ego
	 * @param y_ego
	 * @param yaw_ego
	 * @param cov_xx_ego assumed to be expressed in the global coordinate system
	 * @param cov_xy_ego assumed to be expressed in the global coordinate system
	 * @param cov_yy_ego assumed to be expressed in the global coordinate system
	 * @param x_other
	 * @param y_other
	 * @param yaw_other
	 * @param vx_other
	 * @param vy_other
	 * @param occupancy_model_radius radius of 'ego' agent's circular occupancy model
	 * @param fov_ego total angular field of view of the ego agent
	 */
	HeadingDirectionDisturbance(
		double x_human,
		double y_human,
		double yaw_human,
		double cov_xx_human,
		double cov_xy_human,
		double cov_yy_human,
		double x_robot,
		double y_robot,
		double yaw_robot,
		double vx_robot,
		double vy_robot,
		double human_occupancy_radius = OCCUPANCY_MODEL_RADIUS_DEFAULT,
		double fov_human = FOV_DEFAULT
	);

	/**
	 * Normalizes results to the worst case for the current arrangement.
	 *
	 * After call to this method all scales available by getter methods will be normalized.
	 *
	 * @param other_circumradius radius of the 'other' (robot) taken into calculation of minimum possible distance
	 * between 'ego' (human) and the 'other' (robot)
	 * @param max_speed maximum linear velocity of the 'other' agent (robot)
	 */
	void normalize(double other_circumradius = CIRCUMRADIUS_DEFAULT, double max_speed = MAX_SPEED_DEFAULT);

	double getDirectionScale() const {
		return direction_disturbance_scale_;
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
		return direction_disturbance_scale_ * fov_scale_ * speed_scale_ / distance_scale_;
	}

	/// Computes scale of the direction factor of the disturbance
	static double computeDirectionDisturbance(
		double x_ego,
		double y_ego,
		double yaw_ego,
		double cov_xx_ego,
		double cov_xy_ego,
		double cov_yy_ego,
		double x_other,
		double y_other,
		double yaw_other,
		double occupancy_model_radius = OCCUPANCY_MODEL_RADIUS_DEFAULT
	);

	/// Computes scale of the FOV factor of the disturbance
	static double computeFovScale(double relative_location_angle, double fov_ego = FOV_DEFAULT);

	/// Computes scale of the speed factor of the disturbance
	static double computeSpeedScale(double vel_x_other, double vel_y_other);

	/// Computes scale of the distance factor of the disturbance
	static double computeDistScale(double x_ego, double y_ego, double x_other, double y_other);

protected:
	double direction_disturbance_scale_;
	double fov_scale_;
	double speed_scale_;
	double distance_scale_;

	/**
	 * @defgroup arrangement Current arrangement of agents
	 * @{
	 */
	Vector3d pose_ego_;
	Matrix2d cov_pos_ego_;

	Vector3d pose_other_;
	Vector2d vel_other_;

	double ego_occupancy_model_radius_;
	double fov_ego_;
	/// @}
};

} // namespace social_nav_utils
