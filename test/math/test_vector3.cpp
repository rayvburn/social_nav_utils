#include <gtest/gtest.h>

#include <social_nav_utils/math/vector3.h>
#include <social_nav_utils/math/row_vector3.h>

#include <math.h>

using namespace social_nav_utils;

TEST(TestVector3, constructorColumn) {
	Vector3d v(6.741258963, 3.357241689, M_PI);
	ASSERT_DOUBLE_EQ(v(0), 6.741258963);
	ASSERT_DOUBLE_EQ(v(1), 3.357241689);
	ASSERT_DOUBLE_EQ(v(2), M_PI);
}

TEST(TestVector3, constructorRow) {
	RowVector3d v(6.741258963, 3.357241689, M_PI);
	ASSERT_DOUBLE_EQ(v(0), 6.741258963);
	ASSERT_DOUBLE_EQ(v(1), 3.357241689);
	ASSERT_DOUBLE_EQ(v(2), M_PI);
}

TEST(TestVector3, additionColumn) {
	Vector3d v1(6.741258963, 3.357241689, M_PI);
	Vector3d v2(4.951623847, 7.142753869, M_PI / 2.0);
	Vector3d vresult = v1 + v2;
	ASSERT_DOUBLE_EQ(vresult(0), 11.692882810000000);
	ASSERT_DOUBLE_EQ(vresult(1), 10.499995558000000);
	ASSERT_DOUBLE_EQ(vresult(2),  4.712388980384690);
}

TEST(TestVector3, additionRow) {
	RowVector3d v1(6.741258963, 3.357241689, M_PI);
	RowVector3d v2(4.951623847, 7.142753869, M_PI / 2.0);
	RowVector3d vresult = v1 + v2;
	ASSERT_DOUBLE_EQ(vresult(0), 11.692882810000000);
	ASSERT_DOUBLE_EQ(vresult(1), 10.499995558000000);
	ASSERT_DOUBLE_EQ(vresult(2),  4.712388980384690);
}

TEST(TestVector3, subtractionColumn) {
	Vector3d v1(6.741258963, 3.357241689, M_PI);
	Vector3d v2(4.951623847, 7.142753869, M_PI / 2.0);
	Vector3d vresult = v1 - v2;
	ASSERT_DOUBLE_EQ(vresult(0),  1.789635116000000);
	ASSERT_DOUBLE_EQ(vresult(1), -3.785512180000000);
	ASSERT_DOUBLE_EQ(vresult(2),  1.570796326794897);
}

TEST(TestVector3, subtractionRow) {
	RowVector3d v1(6.741258963, 3.357241689, M_PI);
	RowVector3d v2(4.951623847, 7.142753869, M_PI / 2.0);
	RowVector3d vresult = v1 - v2;
	ASSERT_DOUBLE_EQ(vresult(0),  1.789635116000000);
	ASSERT_DOUBLE_EQ(vresult(1), -3.785512180000000);
	ASSERT_DOUBLE_EQ(vresult(2),  1.570796326794897);
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
