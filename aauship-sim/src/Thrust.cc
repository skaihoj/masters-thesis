#include <stdio.h>
#include <math.h>

#include <iostream>

//#include <eigen3/Eigen/Dense>
#include <eigen/Eigen/Dense>

#include "Thrust.hh"
//#include "RMatrix.cc"

using namespace std;
using namespace Eigen;

Thrust::Thrust()
{
    F = Vector3d::Constant(0.0);
    tau = Matrix<double, 6, 1>::Constant(0.0);
}

void Thrust::Update(Vector3d r, double f, double alfa)
{
    F << f*cos(alfa), f*sin(alfa), 0;

    tau.segment<3>(0) = F;
    tau.segment<3>(3) = r.cross(F);
}

Matrix<double, 6, 1> Thrust::Tau()
{
    return tau;
}