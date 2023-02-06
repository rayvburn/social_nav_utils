#pragma once

#include <math.h>

namespace social_nav_utils {

/**
 * @brief Aim of this class is unification of vector calculation method
 *
 * This is mostly handy when a vector connecting centers of 2 objects (ego and other) is needed
 */
class DistanceVector {
public:
	DistanceVector(double x_ego, double y_ego, double x_other, double y_other) {
		x_ = x_other - x_ego;
		y_ = y_other - y_ego;

		// length of the vector
		length_ = std::sqrt(
			std::pow(x_, 2)
			+ std::pow(y_, 2)
		);

		// direction of vector connecting robot and person (defines where the robot is located in relation to a person [ego agent])
		angle_ = std::atan2(y_, x_);
	}
	inline double getX() const {
		return x_;
	}
	inline double getY() const {
		return y_;
	}
	inline double getLength() const {
		return length_;
	}

	/**
	 * @brief Retrieves direction of vector connecting other (robot) and ego (person)
	 *
	 * Angle defines where the robot is located in relation to a person [ego agent]
	 */
	inline double getAngle() const {
		return angle_;
	}

protected:
	double x_;
	double y_;
	double length_;
	double angle_;
};

} // namespace social_nav_utils
