#pragma once

#include <social_nav_utils/math/vector.h>

#include <cassert>
#include <cstddef>
#include <cmath>

namespace social_nav_utils {

// forward declaration due to a circular dependency
template <typename T>
class Rotation2D;

// Custom classes since Eigen carries too big computational burden
template <typename T>
class Matrix2 {
public:
	// for direct access to storage elements (avoidance of `operator()`)
	friend class Vector2<T>;
	friend class RowVector2<T>;
	friend class Rotation2D<T>;

	Matrix2():
		Matrix2(T(), T(), T(), T())
	{}

	Matrix2(T m11, T m12, T m21, T m22):
		m_{{m11, m12}, {m21, m22}}
	{}

	T determinant() const {
		return m_[0][0] * m_[1][1] - m_[0][1] * m_[1][0];
	}

	Matrix2<T> transpose() const {
		return Matrix2<T>(m_[0][0], m_[1][0], m_[0][1], m_[1][1]);
	}

	Matrix2<T> inverse() const {
		return Matrix2<T>(
			+m_[1][1] / (m_[0][0] * m_[1][1] - m_[0][1] * m_[1][0]),
			-m_[0][1] / (m_[0][0] * m_[1][1] - m_[0][1] * m_[1][0]),
			-m_[1][0] / (m_[0][0] * m_[1][1] - m_[0][1] * m_[1][0]),
			+m_[0][0] / (m_[0][0] * m_[1][1] - m_[0][1] * m_[1][0])
		);
	}

	Matrix2<T> operator*(const Matrix2<T>& other) const {
		return Matrix2<T>(
			other.m_[0][0] * m_[0][0] + other.m_[1][0] * m_[0][1],
			other.m_[0][1] * m_[0][0] + other.m_[1][1] * m_[0][1],
			other.m_[0][0] * m_[1][0] + other.m_[1][0] * m_[1][1],
			other.m_[0][1] * m_[1][0] + other.m_[1][1] * m_[1][1]
		);
	}

	template <typename Tvec>
	Vector2<T> operator*(const Vector2<Tvec>& vec) const {
		return Vector2<T>(
			m_[0][0] * vec.v_[0] + m_[0][1] * vec.v_[1],
			m_[1][0] * vec.v_[0] + m_[1][1] * vec.v_[1]
		);
	}

	template <typename Tmult>
	Matrix2<T> operator*(const Tmult& mult) const {
		return Matrix2<T>(
			mult * m_[0][0],
			mult * m_[0][1],
			mult * m_[1][0],
			mult * m_[1][1]
		);
	}

	Matrix2<T> operator+(const Matrix2<T>& other) const {
		return Matrix2<T>(
			m_[0][0] + other.m_[0][0],
			m_[0][1] + other.m_[0][1],
			m_[1][0] + other.m_[1][0],
			m_[1][1] + other.m_[1][1]
		);
	}

	Matrix2<T> operator-(const Matrix2<T>& other) const {
		return Matrix2<T>(
			m_[0][0] - other.m_[0][0],
			m_[0][1] - other.m_[0][1],
			m_[1][0] - other.m_[1][0],
			m_[1][1] - other.m_[1][1]
		);
	}

	T operator()(size_t row, size_t col) const {
		assert(row < 2);
		assert(col < 2);
		return m_[row][col];
	}

	template <typename Tang>
	static Matrix2<Tang> rotationMatrix(const Tang& angle) {
		return Matrix2<Tang>(
			std::cos(angle), -std::sin(angle),
			std::sin(angle),  std::cos(angle)
		);
	}

protected:
	T m_[2][2];
};

typedef Matrix2<double> Matrix2d;
typedef Matrix2<float> Matrix2f;

} // namespace social_nav_utils
