#pragma once

#include <social_nav_utils/distance_vector.h>

#include <angles/angles.h>

namespace social_nav_utils {

/**
 * @brief Aim of this class is unification of 'relative location' computation method
 *
 * This is mostly handy when a there is a need to define whether 'other' is located on the right or left side
 * compared to the direction (yaw) of the 'ego'
 */
class RelativeLocation {
public:
	RelativeLocation(double x_ego, double y_ego, double yaw_ego, double x_other, double y_other):
		RelativeLocation(DistanceVector(x_ego, y_ego, x_other, y_other), yaw_ego)
	{}

	RelativeLocation(const DistanceVector& dist_vector, double yaw_ego) {
		rel_loc_angle_ = angles::normalize_angle(dist_vector.getAngle() - yaw_ego);
	}

	inline double getAngle() const {
		return rel_loc_angle_;
	}

	/// @brief Check whether 'other' object is located on the left side compared to yaw (direction) of ego
	inline bool isLeftSide() const {
		return getAngle() >= 0.0;
		// otherwise, getAngle() < 0.0, 'other' is on the right side
	}

	/// @brief Check whether 'other' object is located in front of the ego compared to yaw (direction) of the ego
	inline bool isFront() const {
		return std::abs(getAngle() <= M_PI_2);
	}

protected:
	double rel_loc_angle_;
};

} // namespace social_nav_utils
