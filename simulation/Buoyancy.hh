#include <stdio.h>
#include <math.h>

//#include <eigen3/Eigen/Dense>
#include <eigen/Eigen/Dense>

#include "RMatrix.hh"

using namespace std;
using namespace Eigen;

class Buoyancy
{
    private: 
        /// 
        RMatrix RM;

        Matrix<double, 6, 1> gEta;
        Matrix<double, 6, 1> g0;

    public: 
        /// 
        Buoyancy();

        void UpdateGeta(double iAwp, double rho, double g, double nab, double GMT, double GML, double phi, double th);
        void UpdateG0(double rho, double g, Matrix<double, 2, 1> Vi, Matrix<double, 2, 1> xi, Matrix<double, 2, 1> yi);

        Matrix<double, 6, 1> G0();
        Matrix<double, 6, 1> Geta();
};