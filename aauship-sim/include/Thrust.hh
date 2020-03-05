#include <stdio.h>
#include <math.h>

//#include <eigen3/Eigen/Dense>
#include <eigen/Eigen/Dense>

#include "RMatrix.hh"

using namespace std;
using namespace Eigen;

class Thrust
{
    private: 
        /// 
        RMatrix RM;

        Vector3d F;

        Matrix<double, 6, 1> tau;

    public: 
        /// 
        Thrust();

        void Update(Vector3d r, double f, double alfa);

        Matrix<double, 6, 1> Tau();
};