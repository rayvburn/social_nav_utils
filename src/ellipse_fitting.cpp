#include <social_nav_utils/ellipse_fitting.h>

#include<eigen3/Eigen/Eigenvalues>

#include <angles/angles.h>

#include <algorithm>
#include <limits>
#include <map>
#include <numeric>

namespace social_nav_utils {

// This structure is used to store a point
// Reference: https://github.com/gopiraj15/OpenCV-journey/blob/master/TaubinEllipseFit.cpp#L13
struct Point2d {
	//the cooridinates of the point
	double x, y;
	//A constructor to store the point
	Point2d(double nx, double ny) :x(nx), y(ny){}
	Point2d() :x(0.0), y(0.0){}
};

EllipseFitting::EllipseFitting(const std::vector<double>& x, const std::vector<double>& y):
	params_{NAN},
	fallback_(false)
{
	assert(!x.empty());
	assert(!y.empty());
	assert(x.size() == y.size());

	// primitive case
	if (x.size() == 1) {
		fitFallbackSingle(x.at(0), y.at(0));
		return;
	}

	if (fitTaubin(x, y)) {
		return;
	}

	fitFallbackMultiple(x, y);
}

// a.k.a. EllipseFitbyTaubin
// Reference: https://github.com/gopiraj15/OpenCV-journey/blob/master/TaubinEllipseFit.cpp#L82
bool EllipseFitting::fitTaubin(const std::vector<double>& x, const std::vector<double>& y) {
	// prepare inputs
	std::vector<Point2d> pts;
	for (size_t i = 0; i < x.size(); i++) {
		pts.emplace_back(x.at(i), y.at(i));
	}

	// compute, external code

	Eigen::MatrixXd A = Eigen::MatrixXd::Constant(6, 1, 0);

	Eigen::MatrixXd Xm = Eigen::MatrixXd::Constant(pts.size(), 1, 0);
	Eigen::MatrixXd Ym = Eigen::MatrixXd::Constant(pts.size(), 1, 0);

	for (int i = 0; i < pts.size(); i++)
	{
		Xm(i, 0) = pts[i].x;
		Ym(i, 0) = pts[i].y;
	}

	double meanx = 0, meany = 0;
	for (int i = 0; i < pts.size(); i++)
	{
		meanx += Xm(i, 0);
		meany += Ym(i, 0);
	}
	meanx /= pts.size();
	meany /= pts.size();

	Eigen::MatrixXd Zm = Eigen::MatrixXd::Constant(pts.size(), 6, 0);

	for (int i = 0; i < pts.size(); i++)
	{
		Zm(i, 0) = pow(Xm(i, 0) - meanx, 2);
		Zm(i, 1) = pow((Xm(i, 0) - meanx) * (Ym(i, 0) - meany), 1);
		Zm(i, 2) = pow(Ym(i, 0) - meany, 2);
		Zm(i, 3) = Xm(i, 0) - meanx;
		Zm(i, 4) = Ym(i, 0) - meany;
		Zm(i, 5) = 1;
	}
	Eigen::MatrixXd Mm = Eigen::MatrixXd::Constant(6, 6, 0);

	Mm = (Zm.transpose() * Zm) / pts.size();

	Eigen::MatrixXd Pm = Eigen::MatrixXd::Constant(5, 5, 0), Qm = Eigen::MatrixXd::Constant(5, 5, 0);

	Pm(0, 0) = Mm(0, 0) - Mm(0, 5)*Mm(0, 5);
	Pm(0, 1) = Mm(0, 1) - Mm(0, 5)*Mm(1, 5);
	Pm(0, 2) = Mm(0, 2) - Mm(0, 5)*Mm(2, 5);
	Pm(0, 3) = Mm(0, 3);
	Pm(0, 4) = Mm(0, 4);

	Pm(1, 0) = Mm(0, 1) - Mm(0, 5)*Mm(1, 5);
	Pm(1, 1) = Mm(1, 1) - Mm(1, 5)*Mm(1, 5);
	Pm(1, 2) = Mm(1, 2) - Mm(1, 5)*Mm(2, 5);
	Pm(1, 3) = Mm(1, 3);
	Pm(1, 4) = Mm(1, 4);

	Pm(2, 0) = Mm(0, 2) - Mm(0, 5)*Mm(2, 5);
	Pm(2, 1) = Mm(1, 2) - Mm(1, 5)*Mm(2, 5);
	Pm(2, 2) = Mm(2, 2) - Mm(2, 5)*Mm(2, 5);
	Pm(2, 3) = Mm(2, 3);
	Pm(2, 4) = Mm(2, 4);

	Pm(3, 0) = Mm(0, 3);
	Pm(3, 1) = Mm(1, 3);
	Pm(3, 2) = Mm(2, 3);
	Pm(3, 3) = Mm(3, 3);
	Pm(3, 4) = Mm(3, 4);

	Pm(4, 0) = Mm(0, 4);
	Pm(4, 1) = Mm(1, 4);
	Pm(4, 2) = Mm(2, 4);
	Pm(4, 3) = Mm(3, 4);
	Pm(4, 4) = Mm(4, 4);


	Qm(0, 0) = 4 * Mm(0, 5); Qm(0, 1) = 2 * Mm(1, 5);
	Qm(1, 0) = 2 * Mm(1, 5); Qm(1, 1) = Mm(0, 5) + Mm(2, 5); Qm(1, 2) = 2 * Mm(1, 5);
	Qm(2, 1) = 2 * Mm(1, 5); Qm(2, 2) = 4 * Mm(2, 5);
	Qm(3, 3) = 1;
	Qm(4, 4) = 1;

	//Generalized Eigen value problem solver from the Eigen library
	Eigen::GeneralizedSelfAdjointEigenSolver<Eigen::MatrixXd> EigSolver(Pm, Qm);

	EigSolver.compute(Pm, Qm);

	for (int i = 0; i < 5; i++)
	{
		A(i, 0) = EigSolver.eigenvectors()(i,0);
	}


	Eigen::MatrixXd A13 = Eigen::MatrixXd::Constant(3, 1, 0);
	Eigen::MatrixXd M = Eigen::MatrixXd::Constant(3, 1, 0);

	for (int i = 0; i < 3; i++)
	{
		A13(i, 0) = A(i, 0);
		M(i, 0) = Mm(5, i);
	}

	Eigen::MatrixXd tmp = -A13.transpose() * M;

	A(5, 0) = tmp(0, 0);


	double A4 = A(3, 0) - 2 * A(0, 0)*meanx - A(1, 0)*meany;
	double A5 = A(4, 0) - 2 * A(2, 0)*meany - A(1, 0)*meanx;
	double A6 = A(5, 0) + A(0, 0)*pow(meanx, 2) + A(2, 0)*pow(meany, 2) + A(1, 0)*meanx*meany - A(3, 0)*meanx - A(4, 0)*meany;

	A(3,0) = A4;  A(4,0) = A5;  A(5,0) = A6;

	Eigen::JacobiSVD<Eigen::MatrixXd> svd(A);

	//The largest singular value is given as the sqrt of largest Eigen Value of the Symmetric matrix A.t() * A
	//  ||A|| = sqrt(Lambda_max (A.t()*A) )
	double normA = svd.singularValues()[0];

	A /= (-normA);

	// computations finished

	auto parametric = convertConicToParametric(A);
	params_.at(0) = parametric(0, 0);
	params_.at(1) = parametric(1, 0);
	params_.at(2) = parametric(2, 0);
	params_.at(3) = parametric(3, 0);
	params_.at(4) = parametric(4, 0);

	// verify if results are good
	auto nan_it = std::find_if(
		params_.begin(),
		params_.end(),
		[](const double& param) {
			return std::isnan(param);
		}
	);
	bool has_nan = nan_it != params_.end();
	bool any_axis_negligible = params_.at(2) < 1e-06 || params_.at(3) < 1e-06;
	bool coord_out_of_bounds = std::abs(params_.at(0)) >= 1e06 || std::abs(params_.at(1)) >= 1e06;
	bool axis_out_of_bounds = std::abs(params_.at(2)) >= 1e06 || std::abs(params_.at(3)) >= 1e06;
	return !(has_nan || any_axis_negligible || coord_out_of_bounds || axis_out_of_bounds);
}

bool EllipseFitting::fitFallbackSingle(double x, double y) {
	params_.at(0) = x;
	params_.at(1) = y;
	params_.at(2) = FALLBACK_SIZE;
	params_.at(3) = FALLBACK_SIZE;
	params_.at(4) = 0;
	fallback_ = true;
	return true;
}

bool EllipseFitting::fitFallbackMultiple(const std::vector<double>& x, const std::vector<double>& y) {
	std::vector<Point2d> pts;
	for (size_t i = 0; i < x.size(); i++) {
		pts.emplace_back(x.at(i), y.at(i));
	}

	// center of gravity
	std::array<double, 2> cog;
	cog.at(0) = std::accumulate(x.cbegin(), x.cend(), 0.0) / x.size();
	cog.at(1) = std::accumulate(y.cbegin(), y.cend(), 0.0) / y.size();

	// find longest vector connecting points + store their directions
	// map of index and length
	std::map<std::pair<size_t, size_t>, double> v_lengths;
	// map of index and direction
	std::map<std::pair<size_t, size_t>, double> v_dirs;
	for (size_t i = 0; i < x.size(); i++) {
		for (size_t j = 0; j < x.size(); j++) {
			if (i == j) {
				continue;
			}
			std::array<double, 2> vector;
			vector.at(0) = x.at(j) - x.at(i);
			vector.at(1) = y.at(j) - y.at(i);
			double len = std::hypot(vector.at(0), vector.at(1));
			v_lengths.insert({{i, j}, len});
			double dir = std::atan2(vector.at(1), vector.at(0));
			v_dirs.insert({{i, j}, dir});
		}
	}

	// sort lengths, easier to use vector container; ref: https://stackoverflow.com/a/19528891
	std::vector<std::tuple<std::pair<size_t, size_t>, double, double>> v_to_sort;
	// iterating over lengths and directions
	for (
		auto itl = v_lengths.cbegin(), itd = v_dirs.cbegin();
		itl != v_lengths.cend() && itd != v_dirs.cend();
		++itl, ++itd
	) {
		v_to_sort.push_back(
			std::make_tuple(
				std::make_pair(itl->first.first, itl->first.second),
				itl->second,
				itd->second
			)
		);
	}
	std::sort(
		v_to_sort.begin(),
		v_to_sort.end(),
		[=](
			std::tuple<std::pair<size_t, size_t>, double, double>& a,
			std::tuple<std::pair<size_t, size_t>, double, double>& b
		) {
			return std::get<1>(a) < std::get<1>(b);
		}
	);

	std::pair<std::pair<size_t, size_t>, double> v_len_longest = std::make_pair(
		std::get<0>(v_to_sort.back()),
		std::get<1>(v_to_sort.back())
	);
	std::pair<std::pair<size_t, size_t>, double> v_dir_longest = std::make_pair(
		std::get<0>(v_to_sort.back()),
		std::get<2>(v_to_sort.back())
	);

	// create a vector
	std::array<double, 2> v_longest;
	// longest vector data indices
	auto l_index_from = v_len_longest.first.second;
	auto l_index_to = v_len_longest.first.first;
	v_longest.at(0) = x.at(l_index_to) - x.at(l_index_from);
	v_longest.at(1) = y.at(l_index_to) - y.at(l_index_from);

	// create a unit vector perpendicular to the longest
	// vpl - vector perpendicular to the longest
	double vpl_dir = angles::normalize_angle(v_dir_longest.second + M_PI / 2.0);
	// create a vector directed perpendicularly (a.k.a. `perp_longest_v`)
	std::array<double, 2> v_perp_unit;
	// effect of matrix multiplication: rotation matrix by unit vector [1.0; 0.0]
	v_perp_unit.at(0) = +1.0 * std::cos(vpl_dir) - 0.0 * std::sin(vpl_dir);
	v_perp_unit.at(1) = +1.0 * std::sin(vpl_dir) + 0.0 * std::cos(vpl_dir);

	/*
	 * find 2 longest vector projections onto lines perpendicular to the
	 * longest; vectors are computed from the COG
	 */
	std::vector<std::pair<double, bool>> v_perp_projs;
	for (size_t i = 0; i < x.size(); i++) {
		std::array<double, 2> vector;
		vector.at(0) = x.at(i) - cog.at(0);
		vector.at(1) = y.at(i) - cog.at(1);
		auto v_proj = findVectorProjection(vector, v_perp_unit);
		auto v_proj_len = std::hypot(v_proj.at(0), v_proj.at(1));
		// check whether projection points above or below
		double v_proj_dir = std::atan2(v_proj.at(1), v_proj.at(0));
		double v_dirs_diff = angles::normalize_angle(vpl_dir - v_proj_dir);
		// a.k.a. dir_rel
		bool perp_compliant_dir = false;
		if (std::abs(v_dirs_diff) < 1e-03) {
			perp_compliant_dir = true;
		}
		// collect data
		v_perp_projs.emplace_back(v_proj_len, perp_compliant_dir);
	}

	// extract v_projs in the same direction and choose the longest
	double v_perp_proj_length_c_max = -1.0;
	double v_perp_proj_length_nc_max = -1.0;
	for (const auto& v_proj: v_perp_projs) {
		auto perp_compliant_dir = v_proj.second;
		auto length = v_proj.first;
		if (perp_compliant_dir && length > v_perp_proj_length_c_max) {
			v_perp_proj_length_c_max = length;
		} else if (!perp_compliant_dir && length > v_perp_proj_length_nc_max) {
			v_perp_proj_length_nc_max = length;
		}
	}

	// find exact parameters of the ellipse
	// orientation from the longest vector
	double phi1 = v_dir_longest.second;
	double phi2 = angles::normalize_angle(v_dir_longest.second + M_PI);
	double phi = phi1;
	if (std::abs(phi1) > std::abs(phi2)) {
		phi = phi2;
	}

	// semi axes
	double aaxis = v_len_longest.second / 2;
	double baxis = (v_perp_proj_length_c_max + v_perp_proj_length_nc_max) / 2;

	// small hack when there are people in-line only
	if (baxis < 1e-03) {
		baxis = FALLBACK_SIZE;
	}

	params_.at(0) = cog.at(0);
	params_.at(1) = cog.at(1);
	params_.at(2) = aaxis;
	params_.at(3) = baxis;
	params_.at(4) = phi;
	fallback_ = true;
	return true;
}

// refer to @ gopiraj15/OpenCV-journey for original source of this method
Eigen::MatrixXd EllipseFitting::convertConicToParametric(const Eigen::MatrixXd& par) {
	Eigen::MatrixXd ell = Eigen::MatrixXd::Constant(5, 1, 0);

	double thetarad = 0.5*atan2(par(1,0), par(0,0) - par(2,0));
	double cost = cos(thetarad);
	double sint = sin(thetarad);
	double sin_squared = sint*sint;
	double cos_squared = cost*cost;
	double cos_sin = sint*cost;

	double Ao = par(5,0);
	double Au = par(3,0)*cost + par(4,0)*sint;
	double Av = -par(3,0)*sint + par(4,0)*cost;
	double Auu = par(0,0)*cos_squared + par(2,0)*sin_squared + par(1,0)*cos_sin;
	double Avv = par(0, 0)*sin_squared + par(2, 0)*cos_squared - par(1, 0)*cos_sin;

	double tuCentre = -Au / (2*Auu);
	double tvCentre = -Av / (2.*Avv);
	double wCentre = Ao - Auu*tuCentre*tuCentre - Avv*tvCentre*tvCentre;

	double uCentre = tuCentre*cost - tvCentre*sint;
	double vCentre = tuCentre*sint + tvCentre*cost;

	double Ru = -wCentre / Auu;
	double Rv = -wCentre / Avv;

	Ru = sqrt(abs(Ru))*sign(Ru);
	Rv = sqrt(abs(Rv))*sign(Rv);


	double centrex = uCentre;
	double centrey = vCentre;
	double axea = Ru;
	double axeb = Rv;
	double angle = thetarad;

	ell(0, 0) = centrex;
	ell(1, 0) = centrey;
	ell(2, 0) = axea;
	ell(3, 0) = axeb;
	ell(4, 0) = angle;

	return ell;
}

std::array<double, 2> EllipseFitting::findVectorProjection(
	const std::array<double, 2>& v1,
	const std::array<double, 2>& v2
) {
	// ref: https://iq.opengenus.org/dot-product-of-two-vectors-in-cpp/, NOTE: inner_product does not work here
	double dot_product = 0.0;
	for(size_t i = 0; i < v1.size(); i++) {
		dot_product += v1.at(i) * v2.at(i);
	}

	double v2_length = std::hypot(v2.at(0), v2.at(1));

	double mult = dot_product / std::pow(v2_length, 2);
	std::array<double, 2> proj;
	proj.at(0) = mult * v2.at(0);
	proj.at(1) = mult * v2.at(1);
	return proj;
}

} // namespace social_nav_utils
