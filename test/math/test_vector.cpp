#include <gtest/gtest.h>

// matrix and vector headers are in a circular dependency
#include <social_nav_utils/math/matrix.h>

using namespace social_nav_utils;

TEST(TestVector, constructorColumn) {
	Vector2d v(6.741258963, 3.357241689);
	ASSERT_DOUBLE_EQ(v(0), 6.741258963);
	ASSERT_DOUBLE_EQ(v(1), 3.357241689);
}

TEST(TestVector, constructorRow) {
	RowVector2d v(6.741258963, 3.357241689);
	ASSERT_DOUBLE_EQ(v(0), 6.741258963);
	ASSERT_DOUBLE_EQ(v(1), 3.357241689);
}

TEST(TestVector, transposeColumn) {
	Vector2d v(6.741258963, 3.357241689);
	RowVector2d vtranspose = v.transpose();
	ASSERT_DOUBLE_EQ(vtranspose(0), 6.741258963);
	ASSERT_DOUBLE_EQ(vtranspose(1), 3.357241689);
}

TEST(TestVector, transposeRow) {
	RowVector2d v(6.741258963, 3.357241689);
	Vector2d vtranspose = v.transpose();
	ASSERT_DOUBLE_EQ(vtranspose(0), 6.741258963);
	ASSERT_DOUBLE_EQ(vtranspose(1), 3.357241689);
}

TEST(TestVector, rowMultMat) {
	Vector2d v(6.741258963, 3.357241689);
	Matrix2d m(0.098765, 5.123456, 9.951847623, 2.45623789);
	RowVector2d vresult = v.transpose() * m;
	ASSERT_DOUBLE_EQ(vresult(0), 34.076558163991855);
	ASSERT_DOUBLE_EQ(vresult(1), 42.784727923945525);
}

TEST(TestVector, multVectors) {
	Vector2d v1(6.741258963, 3.357241689);
	Vector2d v2(4.951623847, 7.142753869);
	double result1 = v1.transpose() * v2;
	ASSERT_DOUBLE_EQ(result1, 57.360129703266130);

	Matrix2d result2 = v1 * v2.transpose();
	ASSERT_DOUBLE_EQ(result2(0, 0), 33.380178639993290);
	ASSERT_DOUBLE_EQ(result2(0, 1), 48.151153539899180);
	ASSERT_DOUBLE_EQ(result2(1, 0), 16.623798007394956);
	ASSERT_DOUBLE_EQ(result2(1, 1), 23.979951063272843);
}

TEST(TestVector, addVectors) {
	Vector2d v1(6.741258963, 3.357241689);
	Vector2d v2(4.951623847, 7.142753869);
	Vector2d vresult1 = v1 + v2;
	ASSERT_DOUBLE_EQ(vresult1(0), 11.692882810000000);
	ASSERT_DOUBLE_EQ(vresult1(1), 10.499995558000000);

	RowVector2d vresult2 = v1.transpose() + v2.transpose();
	ASSERT_DOUBLE_EQ(vresult2(0), 11.692882810000000);
	ASSERT_DOUBLE_EQ(vresult2(1), 10.499995558000000);

	Matrix2d vresult3 = v1.transpose() + v2;
	ASSERT_DOUBLE_EQ(vresult3(0, 0), 11.692882810000000);
	ASSERT_DOUBLE_EQ(vresult3(0, 1),  8.308865535999999);
	ASSERT_DOUBLE_EQ(vresult3(1, 0), 13.884012832000000);
	ASSERT_DOUBLE_EQ(vresult3(1, 1), 10.499995558000000);

	Matrix2d vresult4 = v1 + v2.transpose();
	ASSERT_DOUBLE_EQ(vresult4(0, 0), 11.692882810000000);
	ASSERT_DOUBLE_EQ(vresult4(0, 1), 13.884012832000000);
	ASSERT_DOUBLE_EQ(vresult4(1, 0),  8.308865535999999);
	ASSERT_DOUBLE_EQ(vresult4(1, 1), 10.499995558000000);
}

TEST(TestVector, subtractVectors) {
	Vector2d v1(6.741258963, 3.357241689);
	Vector2d v2(4.951623847, 7.142753869);
	Vector2d vresult1 = v1 - v2;
	ASSERT_DOUBLE_EQ(vresult1(0), 1.789635116000000);
	ASSERT_DOUBLE_EQ(vresult1(1), -3.785512180000000);

	RowVector2d vresult2 = v1.transpose() - v2.transpose();
	ASSERT_DOUBLE_EQ(vresult2(0),  1.789635116000000);
	ASSERT_DOUBLE_EQ(vresult2(1), -3.785512180000000);

	Matrix2d vresult3 = v1.transpose() - v2;
	ASSERT_DOUBLE_EQ(vresult3(0, 0),  1.789635116000000);
	ASSERT_DOUBLE_EQ(vresult3(0, 1), -1.594382158000000);
	ASSERT_DOUBLE_EQ(vresult3(1, 0), -0.401494906000000);
	ASSERT_DOUBLE_EQ(vresult3(1, 1), -3.785512180000000);

	Matrix2d vresult4 = v1 - v2.transpose();
	ASSERT_DOUBLE_EQ(vresult4(0, 0),  1.789635116000000);
	ASSERT_DOUBLE_EQ(vresult4(0, 1), -0.401494906000000);
	ASSERT_DOUBLE_EQ(vresult4(1, 0), -1.594382158000000);
	ASSERT_DOUBLE_EQ(vresult4(1, 1), -3.785512180000000);
}

TEST(TestVector, scalarMult) {
	Vector2d v(6.741258963, 3.357241689);
	double scalar = 5.963258147;
	Vector2d vresult = v * scalar;
	ASSERT_DOUBLE_EQ(vresult(0), 40.199867432146526);
	ASSERT_DOUBLE_EQ(vresult(1), 20.020098853377290);
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
