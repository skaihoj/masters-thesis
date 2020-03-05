#include <stdio.h>
#include <math.h>

#include <iostream>

//#include <eigen3/Eigen/Dense>
#include <eigen/Eigen/Dense>

#include "Drag.hh"
//#include "RMatrix.cc"

using namespace std;
using namespace Eigen;

Drag::Drag()
{
    DofVr_ = Matrix<double, 6, 6>::Constant(0.0);

    eye = Matrix<double, 6, 6>::Constant(0.0);

    Dn = Matrix<double, 6, 6>::Constant(0.0);

    for(int i = 0; i < eye.rows(); i++)
    {
        eye(i,i) = 1.0;
    }
}

void Drag::Update(Matrix<double, 6, 6> D, Matrix<double, 6, 1> Vr)
{
    Dn = 0*eye;

    DofVr_ = D + Dn;
}

Matrix<double, 6, 6> Drag::DofVr()
{
    return DofVr_;
}