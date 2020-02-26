#include <stdio.h>
#include <math.h>

//#include <eigen3/Eigen/Dense>
#include <eigen/Eigen/Dense>

#include "RMatrix.hh"

using namespace std;
using namespace Eigen;

class Drag
{
    private: 
        /// 
        RMatrix RM;

        Matrix<double, 6, 6> DofVr_;

        Matrix<double, 6, 6> eye;

        Matrix<double, 6, 6> Dn;

    public: 
        /// 
        Drag();

        void Update(Matrix<double, 6, 6> D, Matrix<double, 6, 1> Vr);

        Matrix<double, 6, 6> DofVr();
};