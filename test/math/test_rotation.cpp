#include <gtest/gtest.h>

#include <social_nav_utils/math/rotation.h>

using namespace social_nav_utils;

TEST(TestRotation, ctor) {
	Rotation2Dd r1(M_PI);
	ASSERT_NEAR(r1(0, 0), -1.0, 1e-09);
	ASSERT_NEAR(r1(0, 1),  0.0, 1e-09);
	ASSERT_NEAR(r1(1, 0),  0.0, 1e-09);
	ASSERT_NEAR(r1(1, 1), -1.0, 1e-09);

	Rotation2Dd r2(-M_PI / 4.0);
	ASSERT_NEAR(r2(0, 0),  0.707106781186548, 1e-09);
	ASSERT_NEAR(r2(0, 1),  0.707106781186548, 1e-09);
	ASSERT_NEAR(r2(1, 0), -0.707106781186548, 1e-09);
	ASSERT_NEAR(r2(1, 1),  0.707106781186548, 1e-09);

	Rotation2Dd r3(M_PI / 6.0);
	ASSERT_NEAR(r3(0, 0),  0.866025403784439, 1e-09);
	ASSERT_NEAR(r3(0, 1), -0.500000000000000, 1e-09);
	ASSERT_NEAR(r3(1, 0),  0.500000000000000, 1e-09);
	ASSERT_NEAR(r3(1, 1),  0.866025403784439, 1e-09);
}

TEST(TestRotation, inverse) {
	Rotation2Dd r(M_PI / 6.0);
	Rotation2Dd rresult = r.inverse();
	ASSERT_NEAR(rresult(0, 0),  0.866025403784439, 1e-09);
	ASSERT_NEAR(rresult(0, 1),  0.500000000000000, 1e-09);
	ASSERT_NEAR(rresult(1, 0), -0.500000000000000, 1e-09);
	ASSERT_NEAR(rresult(1, 1),  0.866025403784439, 1e-09);
}

TEST(TestRotation, multMatrix) {
	Rotation2Dd r(M_PI / 6.0);
	Matrix2d m(0.098765, 5.123456, 9.951847623, 2.45623789);
	Matrix2d rresult = r * m;
	ASSERT_NEAR(rresult(0, 0), -4.890390812495229, 1e-09);
	ASSERT_NEAR(rresult(0, 1),  3.208924106171806, 1e-09);
	ASSERT_NEAR(rresult(1, 0),  8.667935356109782, 1e-09);
	ASSERT_NEAR(rresult(1, 1),  4.688892410477887, 1e-09);

	Matrix2d rresult2 = r * m * r.inverse();
	ASSERT_NEAR(rresult2(0, 0), -5.839664731140793, 1e-09);
	ASSERT_NEAR(rresult2(0, 1),  0.333814388513443, 1e-09);
	ASSERT_NEAR(rresult2(1, 0),  5.162206011513442, 1e-09);
	ASSERT_NEAR(rresult2(1, 1),  8.394667621140792, 1e-09);
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
