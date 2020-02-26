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
    public: 
        DynMod();

        void Update(Matrix<double, 6, 1> v, 
                    Matrix<double, 6, 6> M, Matrix<double, 6, 6> C, Matrix<double, 6, 6> D, 
                    Matrix<double, 6, 1> g, Matrix<double, 6, 1> g0, 
                    Matrix<double, 6, 1> tau, Matrix<double, 6, 1> tau_wind, Matrix<double, 6, 1> tau_wave);

        Matrix<double, 6, 1> Vd();
};