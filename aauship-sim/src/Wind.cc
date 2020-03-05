#include <stdio.h>
#include <math.h>

#include <iostream>

//#include <eigen3/Eigen/Dense>
#include <eigen/Eigen/Dense>

#include "Wind.hh"
//#include "RMatrix.cc"

using namespace std;
using namespace Eigen;

Wind::Wind()
{
    Vrw_ = 0.0;
    gamrw_ = 0.0;

    tau = Matrix<double, 6, 1>::Constant(0.0);

    uw = 0.0;
    vw = 0.0;
    urw = 0.0;
    vrw = 0.0;
}

void Wind::UpdateWind(double rhoa, double Vrw, Matrix<double, 6, 1> Ci, double AFw, double ALw, double HFw, double HLw, double Loa)
{
    tau << Ci(0)*AFw, Ci(1)*ALw, Ci(2)*AFw, Ci(3)*ALw*HLw, Ci(4)*AFw*HFw, Ci(5)*ALw*Loa;

    tau = 0.5*rhoa*pow(Vrw, 2)*tau;
}

void Wind::UpdateWindrw(double u, double v, double Vw, double betaw, double psi)
{
    uw = Vw*cos(betaw - psi);
    vw = Vw*sin(betaw - psi);

    urw = u - uw;
    vrw = v - vw;

    Vrw_ = sqrt(pow(urw, 2) + pow(vrw, 2));
    gamrw_ = -atan2(vrw, urw);
}

Matrix<double, 6, 1> Wind::TauWind()
{
    return tau;
}

double Wind::Vrw()
{
    return Vrw_;
}

double Wind::Gamrw()
{
    return gamrw_;
}