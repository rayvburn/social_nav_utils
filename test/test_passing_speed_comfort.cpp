#include <gtest/gtest.h>

#include <social_nav_utils/passing_speed_comfort.h>

using namespace social_nav_utils;

TEST(TestPassingSpeedComfort, simpleModelFarDistances) {
	// expected results based on Matlab implementation

	const double DIST_FAR1 = 1.0;
	const double DIST_FAR2 = 3.5;
	// normalization
	const double DIST_MIN1 = 0.275;
	const double SPD_MAX1 = 0.55;
	const double DIST_MAX1 = 0.808080808080808;
	const double SPD_MIN1 = 0.424242424242424;

	std::function<double(double,double)> calc_fun = &PassingSpeedComfort::computeSpeedComfort;
	auto calc_norm_fun = [&](double comfort) -> double {
		return PassingSpeedComfort::computeSpeedComfortNormalized(
			comfort,
			DIST_MIN1,
			SPD_MAX1,
			DIST_MAX1,
			SPD_MIN1,
			calc_fun
		);
	};

	// Test cases
	// 1A
	double comf1a = calc_fun(DIST_FAR1, 0.55);
	EXPECT_NEAR(comf1a, 6.13204609535257, 1e-03);
	double comf1a_norm = calc_norm_fun(comf1a);
	EXPECT_NEAR(comf1a_norm, 0.995144048655876, 1e-03);
	// 1B
	double comf1b = calc_fun(DIST_FAR2, 0.55);
	EXPECT_NEAR(comf1b, 6.13204609535257, 1e-03);
	double comf1b_norm = calc_norm_fun(comf1b);
	EXPECT_NEAR(comf1b_norm, 0.995144048655876, 1e-03);
	// 2A
	double comf2a = calc_fun(DIST_FAR1, 0.45);
	EXPECT_NEAR(comf2a, 6.13710811849835, 1e-03);
	double comf2a_norm = calc_norm_fun(comf2a);
	EXPECT_NEAR(comf2a_norm, 0.999774187027599, 1e-03);
	// 2B
	double comf2b = calc_fun(DIST_FAR2, 0.45);
	EXPECT_NEAR(comf2b, 6.13710811849835, 1e-03);
	double comf2b_norm = calc_norm_fun(comf2b);
	EXPECT_NEAR(comf2b_norm, 0.999774187027599, 1e-03);
	// 3A
	double comf3a = calc_fun(DIST_FAR1, 1.50);
	EXPECT_NEAR(comf3a, 5.58613076722117, 1e-03);
	double comf3a_norm = calc_norm_fun(comf3a);
	EXPECT_NEAR(comf3a_norm, 0.495805457009383, 1e-03);
	// 3B
	double comf3b = calc_fun(DIST_FAR2, 1.50);
	EXPECT_NEAR(comf3b, 5.58613076722117, 1e-03);
	double comf3b_norm = calc_norm_fun(comf3b);
	EXPECT_NEAR(comf3b_norm, 0.495805457009383, 1e-03);
	// 4A
	double comf4a = calc_fun(DIST_FAR1, 0.10);
	EXPECT_NEAR(comf4a, 6.10905190207311, 1e-03);
	double comf4a_norm = calc_norm_fun(comf4a);
	EXPECT_NEAR(comf4a_norm, 0.974111687964496, 1e-03);
	// 4B
	double comf4b = calc_fun(DIST_FAR2, 0.10);
	EXPECT_NEAR(comf4b, 6.109051902073112, 1e-03);
	double comf4b_norm = calc_norm_fun(comf4b);
	EXPECT_NEAR(comf4b_norm, 0.974111687964496, 1e-03);
}

TEST(TestPassingSpeedComfort, simpleModelCloseDistances) {
	// expected results based on Matlab implementation

	const double DIST_CLOSE1 = 0.30;
	const double DIST_CLOSE2 = 0.60;
	// normalization
	const double DIST_MIN1 = 0.275;
	const double SPD_MAX1 = 0.55;
	const double DIST_MAX1 = 0.808080808080808;
	const double SPD_MIN1 = 0.424242424242424;

	std::function<double(double,double)> calc_fun = &PassingSpeedComfort::computeSpeedComfort;
	auto calc_norm_fun = [&](double comfort) -> double {
		return PassingSpeedComfort::computeSpeedComfortNormalized(
			comfort,
			DIST_MIN1,
			SPD_MAX1,
			DIST_MAX1,
			SPD_MIN1,
			calc_fun
		);
	};

	// 5A
	double comf5a = calc_fun(DIST_CLOSE1, 1.00);
	EXPECT_NEAR(comf5a, 4.522166867882514, 1e-03);
	double comf5a_norm = calc_norm_fun(comf5a);
	EXPECT_NEAR(comf5a_norm, 0.0, 1e-03);
	// 5B
	double comf5b = calc_fun(DIST_CLOSE2, 1.00);
	EXPECT_NEAR(comf5b, 4.522166867882514, 1e-03);
	double comf5b_norm = calc_norm_fun(comf5b);
	EXPECT_NEAR(comf5b_norm, 0.0, 1e-03);
	// 6A
	double comf6a = calc_fun(DIST_CLOSE1, 0.20);
	EXPECT_NEAR(comf6a, 5.250574722690957, 1e-03);
	double comf6a_norm = calc_norm_fun(comf6a);
	EXPECT_NEAR(comf6a_norm, 0.188878587475037, 1e-03);
	// 6B
	double comf6b = calc_fun(DIST_CLOSE2, 0.20);
	EXPECT_NEAR(comf6b, 5.25057472269096, 1e-03);
	double comf6b_norm = calc_norm_fun(comf6b);
	EXPECT_NEAR(comf6b_norm, 0.188878587475037, 1e-03);
	// 7A
	double comf7a = calc_fun(DIST_CLOSE1, 0.05);
	EXPECT_NEAR(comf7a, 5.145201924386866, 1e-03);
	double comf7a_norm = calc_norm_fun(comf7a);
	EXPECT_NEAR(comf7a_norm, 0.092496049759682, 1e-03);
	// 7B
	double comf7b = calc_fun(DIST_CLOSE2, 0.05);
	EXPECT_NEAR(comf7b, 5.145201924386866, 1e-03);
	double comf7b_norm = calc_norm_fun(comf7b);
	EXPECT_NEAR(comf7b_norm, 0.092496049759682, 1e-03);
}

TEST(TestPassingSpeedComfort, simpleModelMixed) {
	const double DIST_MIX1 = 0.72;
	const double DIST_MIX2 = 0.65;
	// normalization
	const double DIST_MIN1 = 0.275;
	const double SPD_MAX1 = 0.55;
	const double DIST_MAX1 = 0.808080808080808;
	const double SPD_MIN1 = 0.424242424242424;

	std::function<double(double,double)> calc_fun = &PassingSpeedComfort::computeSpeedComfort;
	auto calc_norm_fun = [&](double comfort) -> double {
		return PassingSpeedComfort::computeSpeedComfortNormalized(
			comfort,
			DIST_MIN1,
			SPD_MAX1,
			DIST_MAX1,
			SPD_MIN1,
			calc_fun
		);
	};

	// 8A
	double comf8a = calc_fun(DIST_MIX1, 1.00);
	EXPECT_NEAR(comf8a, 5.414083800879388, 1e-03);
	double comf8a_norm = calc_norm_fun(comf8a);
	EXPECT_NEAR(comf8a_norm, 0.338437298536499, 1e-03);
	// 8B
	double comf8b = calc_fun(DIST_MIX2, 1.00);
	EXPECT_NEAR(comf8b, 4.893798923297878, 1e-03);
	double comf8b_norm = calc_norm_fun(comf8b);
	EXPECT_NEAR(comf8b_norm, 0.0, 1e-03);
	// 9A
	double comf9a = calc_fun(DIST_MIX1, 0.30);
	EXPECT_NEAR(comf9a, 5.773752650290714, 1e-03);
	double comf9a_norm = calc_norm_fun(comf9a);
	EXPECT_NEAR(comf9a_norm, 0.667419701977895, 1e-03);
	// 9B
	double comf9b = calc_fun(DIST_MIX2, 0.30);
	EXPECT_NEAR(comf9b, 5.459408388195655, 1e-03);
	double comf9b_norm = calc_norm_fun(comf9b);
	EXPECT_NEAR(comf9b_norm, 0.379894855104714, 1e-03);
	// 10A
	double comf10a = calc_fun(DIST_MIX1, 0.10);
	EXPECT_NEAR(comf10a, 5.746485691596127, 1e-03);
	double comf10a_norm = calc_norm_fun(comf10a);
	EXPECT_NEAR(comf10a_norm, 0.642479122273636, 1e-03);
	// 10B
	double comf10b = calc_fun(DIST_MIX2, 0.10);
	EXPECT_NEAR(comf10b, 5.429240257428766, 1e-03);
	double comf10b_norm = calc_norm_fun(comf10b);
	EXPECT_NEAR(comf10b_norm, 0.352300627294134, 1e-03);

	// normalization 2
	const double DIST_MIN2 = 0.325;
	const double SPD_MAX2 = 0.85;
	const double DIST_MAX2 = 5.0;
	const double SPD_MIN2 = -0.1;
	auto calc_norm_fun2 = [&](double comfort) -> double {
		return PassingSpeedComfort::computeSpeedComfortNormalized(
			comfort,
			DIST_MIN2,
			SPD_MAX2,
			DIST_MAX2,
			SPD_MIN2,
			calc_fun
		);
	};

	// 11A
	double comf11a = calc_fun(DIST_MIX1, 0.30);
	EXPECT_NEAR(comf11a, 5.773752650290714, 1e-03);
	double comf11a_norm = calc_norm_fun2(comf11a);
	EXPECT_NEAR(comf11a_norm, 0.784442451604271, 1e-03);
	// 11B
	double comf11b = calc_fun(DIST_MIX2, 0.30);
	EXPECT_NEAR(comf11b, 5.459408388195655, 1e-03);
	double comf11b_norm = calc_norm_fun2(comf11b);
	EXPECT_NEAR(comf11b_norm, 0.554129269598922, 1e-03);
}

TEST(TestPassingSpeedComfort, splineModel) {
	// expected results based on Matlab implementation

	std::function<double(double,double)> fun_handle = &PassingSpeedComfort::computeSpeedComfortSpline;

	auto fun_norm_handle = [&fun_handle](
		double dist,
		double speed,
		double dist_min,
		double speed_max,
		double dist_max,
		double speed_min
	) -> double {
		return PassingSpeedComfort::computeSpeedComfortSplineNormalized(
			dist,
			speed,
			dist_min,
			speed_max,
			dist_max,
			speed_min,
			fun_handle
		);
	};

	// normalization, case 1: 1-4; case 2: 5-8
	const double DIST_MIN1 = 0.275;
	const double DIST_MIN2 = 0.325;
	const double SPD_MAX1 = 0.55;
	const double SPD_MAX2 = 0.85;
	const double DIST_MAX1 = 5.0;
	const double DIST_MAX2 = 3.0;
	const double SPD_MIN1 = 0.10;
	const double SPD_MIN2 = 0.15;

	// 1
	EXPECT_NEAR(fun_handle(0.50, 0.55), 4.06583751675401, 1e-09);
	EXPECT_NEAR(fun_norm_handle(0.50, 0.55, DIST_MIN1, SPD_MAX1, DIST_MAX1, SPD_MIN1), 0.555272529840711, 1e-09);
	// 2
	EXPECT_NEAR(fun_handle(0.60, 0.50), 5.10466021505376, 1e-09);
	EXPECT_NEAR(fun_norm_handle(0.60, 0.50, DIST_MIN1, SPD_MAX1, DIST_MAX1, SPD_MIN1), 0.754136360455514, 1e-09);
	// 3
	EXPECT_NEAR(fun_handle(0.70, 0.45), 5.58166278281504, 1e-09);
	EXPECT_NEAR(fun_norm_handle(0.70, 0.45, DIST_MIN1, SPD_MAX1, DIST_MAX1, SPD_MIN1), 0.845449881033530, 1e-09);
	// 4
	EXPECT_NEAR(fun_handle(0.40, 0.45), 3.00730481036354, 1e-09);
	EXPECT_NEAR(fun_norm_handle(0.40, 0.45, DIST_MIN1, SPD_MAX1, DIST_MAX1, SPD_MIN1), 0.352635574402219, 1e-09);

	// 5
	EXPECT_NEAR(fun_handle(0.40, 0.55), 2.78004556099381, 1e-09);
	EXPECT_NEAR(fun_norm_handle(0.40, 0.55, DIST_MIN2, SPD_MAX2, DIST_MAX2, SPD_MIN2), 0.299702211900309, 1e-09);
	// 6
	EXPECT_NEAR(fun_handle(2.00, 0.55), 6.38900000000000, 1e-09);
	EXPECT_NEAR(fun_norm_handle(2.00, 0.55, DIST_MIN2, SPD_MAX2, DIST_MAX2, SPD_MIN2), 1.0, 1e-09);
	// 7
	EXPECT_NEAR(fun_handle(2.00, 0.05), 6.38900000000000, 1e-09);
	EXPECT_NEAR(fun_norm_handle(2.00, 0.05, DIST_MIN2, SPD_MAX2, DIST_MAX2, SPD_MIN2), 1.0, 1e-09);
	// 8
	EXPECT_NEAR(fun_handle(0.05, 0.55), -0.205356383170610, 1e-09);
	EXPECT_NEAR(fun_norm_handle(0.05, 0.55, DIST_MIN2, SPD_MAX2, DIST_MAX2, SPD_MIN2), 0.0, 1e-09);

	// 9 negative speed
	EXPECT_NEAR(fun_handle(0.00, -0.25), 0.0, 1e-09);

	// 10 very far distance
	EXPECT_NEAR(fun_handle(20.00, 0.0), 5.25006401114985, 1e-09);

	// NOTE that the model is not perfect for such conditions
	// 11 very far distance, giant speed
	EXPECT_NEAR(fun_handle(20.00, 5.0), -25.2126517491493, 1e-09);
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
