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
    etadd = Matrix<double, 6, 1>::Constant(0.0);

    Jm1 = Matrix<double, 6, 6>::Constant(0.0);
    JmT = Matrix<double, 6, 6>::Constant(0.0);

    Ms = Matrix<double, 6, 6>::Constant(0.0);
    Cs = Matrix<double, 6, 6>::Constant(0.0);
    Ds = Matrix<double, 6, 6>::Constant(0.0);

    Gs = Matrix<double, 6, 1>::Constant(0.0);
    Ts = Matrix<double, 6, 1>::Constant(0.0);
}

void DynMod::UpdateBODY(Matrix<double, 6, 1> v, 
                    Matrix<double, 6, 6> M, Matrix<double, 6, 6> C, Matrix<double, 6, 6> D, 
                    Matrix<double, 6, 1> g, Matrix<double, 6, 1> g0, 
                    Matrix<double, 6, 1> tau, Matrix<double, 6, 1> tau_wind, Matrix<double, 6, 1> tau_wave)
{
    vd = M.inverse()*(- C*v - D*v - g - g0 + tau + tau_wind + tau_wave);
}

void DynMod::UpdateNED(Matrix<double, 6, 6> J, Matrix<double, 6, 6> Jd, Matrix<double, 6, 1> etad,
                    Matrix<double, 6, 6> M, Matrix<double, 6, 6> C, Matrix<double, 6, 6> D, 
                    Matrix<double, 6, 1> g, Matrix<double, 6, 1> g0, 
                    Matrix<double, 6, 1> tau, Matrix<double, 6, 1> tau_wind, Matrix<double, 6, 1> tau_wave)
{
    Jm1 = J.inverse();
    JmT = Jm1.transpose();

    Ms = JmT*M*Jm1;

    Cs = JmT*(C - M*Jm1*Jd)*Jm1;

    Ds = JmT*D*Jm1;

    Gs = JmT*(g + g0);

    Ts = JmT*(tau + tau_wind + tau_wave);

    etadd = Ms.inverse()*(- Cs*etad - Ds*etad - Gs + Ts);
}

Matrix<double, 6, 1> DynMod::Vd()
{
    return vd;
}

Matrix<double, 6, 1> DynMod::Etadd()
{
    return etadd;
}