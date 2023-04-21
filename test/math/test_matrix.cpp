#include <gtest/gtest.h>

#include <social_nav_utils/math/matrix.h>

using namespace social_nav_utils;

TEST(TestMatrix, constructor) {
	Matrix2d m(0.098765, 5.123456, 9.951847623, 2.45623789);
	ASSERT_DOUBLE_EQ(m(0, 0), 0.098765);
	ASSERT_DOUBLE_EQ(m(0, 1), 5.123456);
	ASSERT_DOUBLE_EQ(m(1, 0), 9.951847623);
	ASSERT_DOUBLE_EQ(m(1, 1), 2.45623789);
}

TEST(TestMatrix, determinant) {
	Matrix2d m1(0.098765, 5.123456, 9.951847623, 2.45623789);
	// evaluated with Matlab
	ASSERT_NEAR(m1.determinant(), -50.745263079939240, 1e-09);
}

TEST(TestMatrix, transpose) {
	Matrix2d m(0.098765, 5.123456, 9.951847623, 2.45623789);
	auto mtranspose = m.transpose();
	ASSERT_DOUBLE_EQ(mtranspose(0, 0), 0.098765);
	ASSERT_DOUBLE_EQ(mtranspose(0, 1), 9.951847623);
	ASSERT_DOUBLE_EQ(mtranspose(1, 0), 5.123456);
	ASSERT_DOUBLE_EQ(mtranspose(1, 1), 2.45623789);
}

TEST(TestMatrix, inverse) {
	Matrix2d m(0.098765, 5.123456, 9.951847623, 2.45623789);
	auto minv = m.inverse();
	// evaluated with Matlab
	ASSERT_NEAR(minv(0, 0), -0.048403294040090, 1e-09);
	ASSERT_NEAR(minv(0, 1),  0.100964221861043, 1e-09);
	ASSERT_NEAR(minv(1, 0),  0.196113824601181, 1e-09);
	ASSERT_NEAR(minv(1, 1), -0.001946290037839, 1e-09);
}

TEST(TestMatrix, operatorMultMat) {
	Matrix2d m1(0.098765, 5.123456, 9.951847623, 2.45623789);
	Matrix2d m2(7.741963852, 5.24863179, 1.987123654, 0.397182645);
	auto mresult = m1 * m2;
	// evaluated with Matlab
	ASSERT_NEAR(mresult(0, 0), 10.945575667671003, 1e-09);
	ASSERT_NEAR(mresult(0, 1),  2.553328924360470, 1e-09);
	ASSERT_NEAR(mresult(1, 0), 81.927692968948180, 1e-09);
	ASSERT_NEAR(mresult(1, 1), 53.209158865213160, 1e-09);
}

TEST(TestMatrix, operatorMultVec) {
	Matrix2d m(0.098765, 5.123456, 9.951847623, 2.45623789);
	Vector2d v(6.741258963, 3.357241689);
	Vector2d result = m * v;
	// evaluated with Matlab
	ASSERT_NEAR(result(0), 17.866480516437880, 1e-09);
	ASSERT_NEAR(result(1), 75.334166229368390, 1e-09);
}

TEST(TestMatrix, operatorMultScalar) {
	Matrix2d m1(0.098765, 5.123456, 9.951847623, 2.45623789);
	auto mresult = m1 * 9.987654321;
	// evaluated with Matlab
	ASSERT_NEAR(mresult(0, 0),  0.986430679013565, 1e-09);
	ASSERT_NEAR(mresult(0, 1), 51.171307456853380, 1e-09);
	ASSERT_NEAR(mresult(1, 0), 99.395613913789550, 1e-09);
	ASSERT_NEAR(mresult(1, 1), 24.532054975462426, 1e-09);
}

TEST(TestMatrix, operatorAdd) {
	Matrix2d m1(0.098765, 5.123456, 9.951847623, 2.45623789);
	Matrix2d m2(7.741963852, 5.24863179, 1.987123654, 0.397182645);
	auto mresult = m1 + m2;
	// evaluated with Matlab
	ASSERT_NEAR(mresult(0, 0),  7.840728852000000, 1e-09);
	ASSERT_NEAR(mresult(0, 1), 10.372087790000000, 1e-09);
	ASSERT_NEAR(mresult(1, 0), 11.938971277000000, 1e-09);
	ASSERT_NEAR(mresult(1, 1),  2.853420535000000, 1e-09);
}

TEST(TestMatrix, operatorSubtract) {
	Matrix2d m1(0.098765, 5.123456, 9.951847623, 2.45623789);
	Matrix2d m2(7.741963852, 5.24863179, 1.987123654, 0.397182645);
	auto mresult = m1 - m2;
	// evaluated with Matlab
	ASSERT_NEAR(mresult(0, 0), -7.643198851999999, 1e-09);
	ASSERT_NEAR(mresult(0, 1), -0.125175790000000, 1e-09);
	ASSERT_NEAR(mresult(1, 0),  7.964723969000001, 1e-09);
	ASSERT_NEAR(mresult(1, 1),  2.059055245000000, 1e-09);
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
