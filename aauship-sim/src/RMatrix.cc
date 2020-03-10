#include <stdio.h>
#include <math.h>

#include "RMatrix.hh"

//#include <eigen3/Eigen/Dense>
#include <eigen/Eigen/Dense>

#include <iostream>

using namespace std;
using namespace Eigen;

/// Constructor 
RMatrix::RMatrix()
{
    M = Matrix3d::Constant(0.0);

    I << 1, 0, 0,
         0, 1, 0,
         0, 0, 1;

    Z = Matrix3d::Constant(0.0);

    Vec = Vector3d::Constant(0.0);
    
    pi = 3.14159265358979323846;

    J = Matrix<double, 6, 6>::Constant(0.0);
}

Matrix3d RMatrix::Rx(double x)
{
    M << 1,   0,       0,
         0, cos(x), -sin(x),
         0, sin(x),  cos(x);

    return M;
}

Matrix3d RMatrix::Ry(double x)
{
    M << cos(x), 0, sin(x),
           0,    1,   0,
        -sin(x), 0, cos(x);
    
    return M;
}

Matrix3d RMatrix::Rz(double x)
{
    M << cos(x), -sin(x), 0,
         sin(x),  cos(x), 0,
           0,       0,    1;
    
    return M;
}

Matrix3d RMatrix::GetRotMat(Vector3d a, Vector3d b)
{
    v = a.cross(b);
    s = v.norm();
    c = a.dot(b);
    
    vx << 0, -v(2), v(1),
          v(2), 0, -v(0),
         -v(1), v(0), 0;

    R = I + vx + (vx*vx) * ((1 - c) / pow(s, 2));

    if(isnan(R(0,0)) || isnan(R(0,1)) || isnan(R(0,2)) || 
       isnan(R(1,0)) || isnan(R(1,1)) || isnan(R(1,2)) || 
       isnan(R(2,0)) || isnan(R(2,1)) || isnan(R(2,2))
             )
    {
        if((Sign(a(0)) == Sign(b(0))) && (Sign(a(1)) == Sign(b(1))) && (Sign(a(2)) == Sign(b(2))))
        {
            R = I;
        }
        else
        {
            R = -I;
        }
    }

    return R;
}

int RMatrix::Sign(double x)
{
    if(x > 0)
    {
        return 1;
    }
    else if (x == 0)
    {
        return 0;
    }
    else
    {
        return -1;
    }
}

Vector3d RMatrix::R_ZYX_2EUL(Matrix3d rm)
{
    thY = -asin(rm(2, 0));
    
    if(rm(2,0) == 1)
    {
        thX = 0.0;
        thZ = atan2(-rm(0,1), -rm(0,2));
    }
    else if(rm(2,0) == -1)
    {
        thX = 0.0;
        thZ = atan2(rm(0,1), rm(0,2));
    }
    else
    {
        thX = atan2(rm(1,0), rm(0,0));
        thZ = atan2(rm(2,1), rm(2,2));
    }

    Vec << thX, thY, thZ;

    return Vec;
}

Matrix3d RMatrix::SofL(Vector3d L)
{
    M <<    0,  -L(2),  L(1),
         L(2),      0, -L(0),
        -L(1),   L(0),     0;
    
    return M;
}

Matrix<double, 6, 6> RMatrix::JofETa(double phi, double th, double psi)
{
    R = Rz(psi)*Ry(th)*Rx(phi);

    M << 1, sin(phi)*tan(th), cos(phi)*tan(th),
         0, cos(phi)        , -sin(phi),
         0, sin(phi)/cos(th), cos(phi)/cos(th);

    J.block<3,3>(0,0) = R;
    J.block<3,3>(0,3) = Z;
    J.block<3,3>(3,0) = Z;
    J.block<3,3>(3,3) = M;

    return J;
}

Matrix<double, 6, 1> RMatrix::NED2Global(Matrix<double, 6, 1> Eta)
{
    J.block<3,3>(0,0) = Rx(pi);
    J.block<3,3>(0,3) = Z;
    J.block<3,3>(3,0) = Z;
    J.block<3,3>(3,3) = Rx(pi);

    return J*Eta;
}


