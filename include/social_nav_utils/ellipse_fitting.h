#pragma once

#include <eigen3/Eigen/Core>

#include<array>
#include<vector>

namespace social_nav_utils {

/*
 * A class that performs ellipse fitting

 * Taubin method implemented by Gopiraj @ https://github.com/gopiraj15. Source code provided at:
 * https://github.com/gopiraj15/OpenCV-journey/blob/master/TaubinEllipseFit.cpp
 *
 * The method mentioned above seems to be the best that works well for ellipses determined by less than 5 points.
 */
class EllipseFitting {
public:
	// approximation of space occupied by, e.g., a human
	static constexpr auto FALLBACK_SIZE = 0.28;

	/// Performs ellipse fitting to the points given by @ref x and @ref y
	EllipseFitting(const std::vector<double>& x, const std::vector<double>& y);

	inline double getCenterX() const {
		return params_.at(0);
	}
	inline double getCenterY() const {
		return params_.at(1);
	}
	inline double getSemiAxisMajor() const {
		return params_.at(2);
	}
	inline double getSemiAxisMinor() const {
		return params_.at(3);
	}
	inline double getOrientation() const {
		return params_.at(4);
	}
	inline bool usedFallback() const {
		return fallback_;
	}

protected:
	std::array<double, 5> params_;

	/// Whether solver-based method or fallback one was used
	bool fallback_;

	/// Performs ellipse fitting, establishes conic representation
	/**
	 * This function fits an Ellipse to the given set of points.
	 *
	 * The resulting Conic may not be Ellipse always.
	 *
	 * @copyright (C) 2018 Gopiraj @ https://github.com/gopiraj15
	 * https://github.com/gopiraj15/OpenCV-journey/blob/master/TaubinEllipseFit.cpp
	 */
	bool fitTaubin(const std::vector<double>& x, const std::vector<double>& y);

	/// Performs heuristic ellipse creation around a single point
	bool fitFallbackSingle(double x, double y);

	/// Performs heuristic ellipse fitting once main solver returns NaNs
	bool fitFallbackMultiple(const std::vector<double>& x, const std::vector<double>& y);

	/**
	 * Converts The Conic in the form [A B C D E F] into an Ellipse of the form [centrex centrey axea axeb angle]
	 *
	 * @copyright (C) 2018 Gopiraj @ https://github.com/gopiraj15
	 * https://github.com/gopiraj15/OpenCV-journey/blob/master/TaubinEllipseFit.cpp
	 */
	Eigen::MatrixXd convertConicToParametric(const Eigen::MatrixXd& par);

	/**
	 * @brief equivalent to the sign function from Matlab
	 *
	 * @copyright (C) 2018 Gopiraj @ https://github.com/gopiraj15
	 */
	template <typename T>
	int sign(const T &val) { return (val > 0) - (val < 0); }

	/**
	 * @brief equivalent to the sign function from Matlab
	 *
	 * @copyright (C) 2018 Gopiraj @ https://github.com/gopiraj15
	 */
	template <typename T>
	std::vector<int> sign(const std::vector<T> &v) {
		std::vector<int> r(v.size());
		std::transform(v.begin(), v.end(), r.begin(), (int(*)(const T&))sign);
		return r;
	}
}; // class EllipseFitting

} // namespace social_nav_utils
