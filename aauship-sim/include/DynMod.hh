#include <stdio.h>
#include <math.h>

//#include <eigen3/Eigen/Dense>
#include <eigen/Eigen/Dense>

using namespace std;
using namespace Eigen;

class DynMod
{
    private:
        Matrix<double, 6, 1> vd;
        Matrix<double, 6, 1> etadd;

        Matrix<double, 6, 6> Jm1;
        Matrix<double, 6, 6> JmT;

        Matrix<double, 6, 6> Ms;
        Matrix<double, 6, 6> Cs;
        Matrix<double, 6, 6> Ds;

        Matrix<double, 6, 1> Gs;
        Matrix<double, 6, 1> Ts;
    public: 
        DynMod();

        void UpdateBODY(Matrix<double, 6, 1> v, 
                    Matrix<double, 6, 6> M, Matrix<double, 6, 6> C, Matrix<double, 6, 6> D, 
                    Matrix<double, 6, 1> g, Matrix<double, 6, 1> g0, 
                    Matrix<double, 6, 1> tau, Matrix<double, 6, 1> tau_wind, Matrix<double, 6, 1> tau_wave);

        void UpdateNED(Matrix<double, 6, 6> J, Matrix<double, 6, 6> Jd, Matrix<double, 6, 1> etad,
                  Matrix<double, 6, 6> M, Matrix<double, 6, 6> C, Matrix<double, 6, 6> D, 
                  Matrix<double, 6, 1> g, Matrix<double, 6, 1> g0, 
                  Matrix<double, 6, 1> tau, Matrix<double, 6, 1> tau_wind, Matrix<double, 6, 1> tau_wave);

        Matrix<double, 6, 1> Vd();
        Matrix<double, 6, 1> Etadd();
};