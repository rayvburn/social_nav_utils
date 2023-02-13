#pragma once

#include <vector>
#include <stdexcept>
#include <math.h>

namespace social_nav_utils {

/**
 * @brief This is a port of Matlab implementation by preethamam
 * Available at https://github.com/preethamam/Line2Line-IntersectionPoint
 */
class LinesIntersection {
public:
	/**
	 * @brief Constructor that performs all computations
	 * 
	 * @param l1x Line 1 x1 and x2 coordinates [x1, x2]
	 * @param l1y Line 1 y1 and y2 coordinates [y1, y2]
	 * @param l2x Line 2 x3 and x4 coordinates [x3, x4]
	 * @param l2y Line 2 y3 and y4 coordinates [y3, y4]
	 */
	LinesIntersection(
		const std::vector<double>& l1x, 
		const std::vector<double>& l1y, 
		const std::vector<double>& l2x, 
		const std::vector<double>& l2y
	):
		xi_(NAN),
		yi_(NAN)
	{
		if (l1x.size() < 2 || l1y.size() < 2 || l2x.size() < 2 || l2y.size() < 2) {
			throw std::runtime_error("Not enough input data to find intersection point");
		}

		double x1 = l1x.at(0);
		double y1 = l1y.at(0);
		double x2 = l1x.at(1);
		double y2 = l1y.at(1);
		double x3 = l2x.at(0);
		double y3 = l2y.at(0);
		double x4 = l2x.at(1);
		double y4 = l2y.at(1);

		// Line segments intersect parameters
		double u = ((x1-x3)*(y1-y2) - (y1-y3)*(x1-x2)) / ((x1-x2)*(y3-y4)-(y1-y2)*(x3-x4));
		double t = ((x1-x3)*(y3-y4) - (y1-y3)*(x3-x4)) / ((x1-x2)*(y3-y4)-(y1-y2)*(x3-x4));

		// Check if intersection exists, if so then store the value
		if ((u >= 0 && u <= 1.0) && (t >= 0 && t <= 1.0)) {
			xi_ = ((x3 + u * (x4-x3)) + (x1 + t * (x2-x1))) / 2;
			yi_ = ((y3 + u * (y4-y3)) + (y1 + t * (y2-y1))) / 2;
			return;
		}

		xi_ = NAN;
		yi_ = NAN;
	}

	/// Returns interection point's x coordinate (NaN if no interesction)
	double getX() const {
		return xi_;
	}

	/// Returns interection point's y coordinate (NaN if no interesction)
	double getY() const {
		return yi_;
	}

protected:
	double xi_;
	double yi_;
};

} // namespace social_nav_utils
