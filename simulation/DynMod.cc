#include <stdio.h>
#include <math.h>

//#include <eigen3/Eigen/Dense>
#include <eigen/Eigen/Dense>

#include "DynMod.hh"

using namespace std;
using namespace Eigen;

DynMod::DynMod()
{
    vd = Matrix<double, 6, 1>::Constant(0.0);
}

void DynMod::Update(Matrix<double, 6, 1> v, 
                    Matrix<double, 6, 6> M, Matrix<double, 6, 6> C, Matrix<double, 6, 6> D, 
                    Matrix<double, 6, 1> g, Matrix<double, 6, 1> g0, 
                    Matrix<double, 6, 1> tau, Matrix<double, 6, 1> tau_wind, Matrix<double, 6, 1> tau_wave)
{
    vd = M.inverse()*(- C*v - D*v - g - g0 + tau + tau_wind + tau_wave);
}

Matrix<double, 6, 1> DynMod::Vd()
{
    return vd;
}