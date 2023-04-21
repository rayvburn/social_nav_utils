#pragma once

#include <cassert>
#include <cstddef>

namespace social_nav_utils {

/**
 * @brief Class handling 3-element row vectors
 */
template <typename T>
class RowVector3 {
public:
	RowVector3(): RowVector3(T(), T(), T()) {}

	RowVector3(T v1, T v2, T v3): v_{v1, v2, v3} {}

	template <typename Tvec>
	RowVector3<T> operator+(const RowVector3<Tvec>& other) {
		return RowVector3<T>(
			v_[0] + other.v_[0],
			v_[1] + other.v_[1],
			v_[2] + other.v_[2]
		);
	}

	template <typename Tvec>
	RowVector3<T> operator-(const RowVector3<Tvec>& other) {
		return RowVector3<T>(
			v_[0] - other.v_[0],
			v_[1] - other.v_[1],
			v_[2] - other.v_[2]
		);
	}

	T operator()(size_t index) const {
		assert(index < 3);
		return v_[index];
	}

protected:
	T v_[3];
};

typedef RowVector3<double> RowVector3d;
typedef RowVector3<float> RowVector3f;

} // namespace social_nav_utils
