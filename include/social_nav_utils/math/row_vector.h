#pragma once

#include <cassert>
#include <cstddef>

namespace social_nav_utils {

// forward declarations due to a circular dependencies
template <typename T>
class Vector2;

template <typename T>
class Matrix2;

/**
 * @brief Class handling row vectors
 */
template <typename T>
class RowVector2 {
public:
	// for direct access to storage elements (avoidance of `operator()`)
	friend class Vector2<T>;
	friend class Matrix2<T>;

	RowVector2(): RowVector2(T(), T()) {}

	RowVector2(T v1, T v2): v_{v1, v2} {}

	Vector2<T> transpose() const {
		return Vector2<T>(v_[0], v_[1]);
	}

	template <typename Tmat>
	RowVector2<T> operator*(const Matrix2<Tmat>& matrix) const {
		return RowVector2<T>(
			matrix.m_[0][0] * v_[0] + matrix.m_[1][0] * v_[1],
			matrix.m_[0][1] * v_[0] + matrix.m_[1][1] * v_[1]
		);
	}

	template <typename Tvec>
	T operator*(const Vector2<Tvec>& other) const {
		return static_cast<T>(other.v_[0] * v_[0] + other.v_[1] * v_[1]);
	}

	template <typename Tmult>
	RowVector2<T> operator*(const Tmult& mult) const {
		return RowVector2<T>(mult * v_[0], mult * v_[1]);
	}

	template <typename Trvec>
	RowVector2<T> operator+(const RowVector2<Trvec>& other) const {
		return RowVector2<T>(
			v_[0] + other.v_[0],
			v_[1] + other.v_[1]
		);
	}

	template <typename Tvec>
	Matrix2<T> operator+(const Vector2<Tvec>& other) const {
		return Matrix2<T>(
			v_[0] + other.v_[0], v_[1] + other.v_[0],
			v_[0] + other.v_[1], v_[1] + other.v_[1]
		);
	}

	template <typename Trvec>
	RowVector2<T> operator-(const RowVector2<Trvec>& other) const {
		return RowVector2<T>(
			v_[0] - other.v_[0],
			v_[1] - other.v_[1]
		);
	}

	template <typename Tvec>
	Matrix2<T> operator-(const Vector2<Tvec>& other) const {
		return Matrix2<T>(
			v_[0] - other.v_[0], v_[1] - other.v_[0],
			v_[0] - other.v_[1], v_[1] - other.v_[1]
		);
	}

	T operator()(size_t index) const {
		assert(index < 2);
		return v_[index];
	}

protected:
	T v_[2];
};

typedef RowVector2<double> RowVector2d;
typedef RowVector2<float> RowVector2f;

} // namespace social_nav_utils
