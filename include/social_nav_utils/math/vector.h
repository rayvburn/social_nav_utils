#pragma once

#include <social_nav_utils/math/row_vector.h>

#include <cassert>
#include <cstddef>

namespace social_nav_utils {

// forward declaration due to a circular dependency
template <typename T>
class Matrix2;

/**
 * @brief Class handling typical column vectors
 */
template <typename T>
class Vector2 {
public:
	// for direct access to storage elements (avoidance of `operator()`)
	friend class RowVector2<T>;
	friend class Matrix2<T>;

	Vector2(): Vector2(T(), T()) {}

	Vector2(T v1, T v2): v_{v1, v2} {}

	RowVector2<T> transpose() const {
		return RowVector2<T>(v_[0], v_[1]);
	}

	template <typename Trvec>
	Matrix2<T> operator*(const RowVector2<Trvec>& other) const {
		return Matrix2<T>(
			v_[0] * other.v_[0], v_[0] * other.v_[1],
			v_[1] * other.v_[0], v_[1] * other.v_[1]
		);
	}

	template <typename Tmult>
	Vector2<T> operator*(const Tmult& mult) const {
		return Vector2<T>(mult * v_[0], mult * v_[1]);
	}

	Vector2<T> operator+(const Vector2<T>& other) const {
		return Vector2<T>(
			v_[0] + other.v_[0],
			v_[1] + other.v_[1]
		);
	}

	template <typename Trvec>
	Matrix2<T> operator+(const RowVector2<Trvec>& other) const {
		return Matrix2<T>(
			v_[0] + other.v_[0], v_[0] + other.v_[1],
			v_[1] + other.v_[0], v_[1] + other.v_[1]
		);
	}

	Vector2<T> operator-(const Vector2<T>& other) const {
		return Vector2<T>(
			v_[0] - other.v_[0],
			v_[1] - other.v_[1]
		);
	}

	template <typename Trvec>
	Matrix2<T> operator-(const RowVector2<Trvec>& other) const {
		return Matrix2<T>(
			v_[0] - other.v_[0], v_[0] - other.v_[1],
			v_[1] - other.v_[0], v_[1] - other.v_[1]
		);
	}

	T operator()(size_t index) const {
		assert(index < 2);
		return v_[index];
	}

protected:
	T v_[2];
};

typedef Vector2<double> Vector2d;
typedef Vector2<float> Vector2f;

} // namespace social_nav_utils
