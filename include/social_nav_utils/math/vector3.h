#pragma once

#include <cassert>
#include <cstddef>

namespace social_nav_utils {

/**
 * @brief Class handling typical 3-element column vectors
 */
template <typename T>
class Vector3 {
public:
	Vector3(): Vector3(T(), T(), T()) {}

	Vector3(T v1, T v2, T v3): v_{v1, v2, v3} {}

	template <typename Tvec>
	Vector3<T> operator+(const Vector3<Tvec>& other) {
		return Vector3<T>(
			v_[0] + other.v_[0],
			v_[1] + other.v_[1],
			v_[2] + other.v_[2]
		);
	}

	template <typename Tvec>
	Vector3<T> operator-(const Vector3<Tvec>& other) {
		return Vector3<T>(
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

typedef Vector3<double> Vector3d;
typedef Vector3<float> Vector3f;

} // namespace social_nav_utils
