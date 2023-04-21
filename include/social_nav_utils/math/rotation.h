#pragma once

#include <social_nav_utils/math/matrix.h>

#include <cmath>

namespace social_nav_utils {

// Class representing a rotation matrix (rotation around Z axis)
template <typename T>
class Rotation2D: public Matrix2<T> {
public:
	// for direct access to storage elements (avoidance of `operator()`)
	friend class Matrix2<T>;
	friend class Vector2<T>;
	friend class RowVector2<T>;

	Rotation2D(T angle = T()):
		Matrix2<T>(
			std::cos(angle), -std::sin(angle),
			std::sin(angle),  std::cos(angle)
		)
	{}

	Rotation2D(const Matrix2<T>& mat):
		Rotation2D(mat.m_[0][0], mat.m_[0][1], mat.m_[1][0], mat.m_[1][1])
	{}

	// Returning Rotation2D here disallows executing, e.g., R * MAT * R.inverse()
	Matrix2<T> inverse() const {
		return this->transpose();
	}

protected:
	// to prevent creating strange objects
	Rotation2D(T m11, T m12, T m21, T m22):
		Matrix2<T>(m11, m12, m21, m22)
	{}
};

typedef Rotation2D<double> Rotation2Dd;
typedef Rotation2D<float> Rotation2Df;

} // namespace social_nav_utils
