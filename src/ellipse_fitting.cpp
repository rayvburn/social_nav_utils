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
	bool has_nan = std::find(params_.begin(), params_.end(), NAN) != params_.end();
	bool both_axes_negligible = params_.at(2) < 1e-03 && params_.at(3) < 1e-03;
	bool coord_out_of_bounds = std::abs(params_.at(0)) >= 1e06 || std::abs(params_.at(1)) >= 1e06;
	bool axis_out_of_bounds = std::abs(params_.at(2)) >= 1e06 || std::abs(params_.at(3)) >= 1e06;
	return !(has_nan || both_axes_negligible || coord_out_of_bounds || axis_out_of_bounds);
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
	// TODO
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

} // namespace social_nav_utils
