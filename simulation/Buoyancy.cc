#include <stdio.h>
#include <math.h>

#include <iostream>

//#include <eigen3/Eigen/Dense>
#include <eigen/Eigen/Dense>

#include "Buoyancy.hh"
//#include "RMatrix.cc"

using namespace std;
using namespace Eigen;

Buoyancy::Buoyancy()
{
    gEta = Matrix<double, 6, 1>::Constant(0.0);
    g0 = Matrix<double, 6, 1>::Constant(0.0);

}

void Buoyancy::UpdateGeta(double iAwp, double rho, double g, double nab, double GMT, double GML, double phi, double th)
{
    gEta << -iAwp*sin(th),
            iAwp*cos(th)*sin(phi),
            iAwp*cos(th)*cos(phi),
            nab*GMT*sin(phi)*cos(th)*cos(phi),
            nab*GML*sin(th)*cos(th)*cos(phi),
            nab*(-GML*cos(th) + GMT)*sin(phi)*sin(th);

    gEta = rho*g*gEta;
}

void Buoyancy::UpdateG0(double rho, double g, Matrix<double, 2, 1> Vi, Matrix<double, 2, 1> xi, Matrix<double, 2, 1> yi)
{
    g0 << 0,
          0,
          Vi.sum(),
          -(yi.transpose()*Vi),
          -(xi.transpose()*Vi),
          0;

    g0 = rho*g*g0;
}

Matrix<double, 6, 1> Buoyancy::Geta()
{
    return gEta;
}

Matrix<double, 6, 1> Buoyancy::G0()
{
    return g0;
}