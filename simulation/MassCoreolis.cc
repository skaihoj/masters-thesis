#include <stdio.h>
#include <math.h>

#include <iostream>

//#include <eigen3/Eigen/Dense>
#include <eigen/Eigen/Dense>

#include "MassCoreolis.hh"
//#include "RMatrix.cc"

using namespace std;
using namespace Eigen;

MassCoreolis::MassCoreolis()
{
    MRB = Matrix<double, 6, 6>::Constant(0.0);
    M = Matrix<double, 6, 6>::Constant(0.0);

    v1 = Vector3d::Constant(0.0);
    v2 = Vector3d::Constant(0.0);

    CRB = Matrix<double, 6, 6>::Constant(0.0);
    CA = Matrix<double, 6, 6>::Constant(0.0);
    C = Matrix<double, 6, 6>::Constant(0.0);

    A11 = Matrix3d::Constant(0.0);
    A12 = Matrix3d::Constant(0.0);
    A21 = Matrix3d::Constant(0.0);
    A22 = Matrix3d::Constant(0.0);

    eye << 1, 0, 0,
           0, 1, 0,
           0, 0, 1;
}

void MassCoreolis::Update(double m, Vector3d rgb, Matrix3d Ib, Matrix<double, 6, 6> MA, Matrix<double, 6, 1> v)
{
    // Compute added mass
    
    MRB.block<3,3>(0,0) = m*eye;
    MRB.block<3,3>(0,3) = -m*RM.SofL(rgb);
    MRB.block<3,3>(3,0) = m*RM.SofL(rgb);
    MRB.block<3,3>(3,3) = Ib;

    M = MRB + MA;

    // Compute coreolis 

    v1 << v(0), v(1), v(2);
    v2 << v(3), v(4), v(5);

    CRB.block<3,3>(0,0) = m*RM.SofL(v2);
    CRB.block<3,3>(0,3) = -m*RM.SofL(v2)*RM.SofL(rgb);
    CRB.block<3,3>(3,0) = m*RM.SofL(rgb)*RM.SofL(v2);
    CRB.block<3,3>(3,3) = -RM.SofL(Ib*v2);

    A11 = MA.block<3,3>(0,0);
    A12 = MA.block<3,3>(0,3);
    A21 = MA.block<3,3>(3,0);
    A22 = MA.block<3,3>(3,3);

    CA.block<3,3>(0,0) = eye;
    CA.block<3,3>(0,3) = -RM.SofL(A11*v1 + A12*v2);
    CA.block<3,3>(3,0) = -RM.SofL(A11*v1 + A12*v2);
    CA.block<3,3>(3,3) = -RM.SofL(A21*v1 + A22*v2);

    C = CRB + CA;
}

Matrix<double, 6, 6> MassCoreolis::Mass()
{
    return M;
}

Matrix<double, 6, 6> MassCoreolis::Coreolis()
{
    return C;
}