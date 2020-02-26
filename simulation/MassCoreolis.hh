#include <stdio.h>
#include <math.h>

//#include <eigen3/Eigen/Dense>
#include <eigen/Eigen/Dense>

#include "RMatrix.hh"

using namespace std;
using namespace Eigen;

class MassCoreolis
{
    private: 
        /// 
        RMatrix RM;

        Matrix<double, 6, 6> MRB;
        Matrix<double, 6, 6> M;

        Matrix3d eye;

        Vector3d v1;
        Vector3d v2;

        Matrix<double, 6, 6> CRB;
        Matrix<double, 6, 6> CA;
        Matrix<double, 6, 6> C;

        Matrix3d A11;
        Matrix3d A12;
        Matrix3d A21;
        Matrix3d A22;

    public: 
        /// 
        MassCoreolis();

        void Update(double m, Vector3d rgb, Matrix3d Ib, Matrix<double, 6, 6> MA, Matrix<double, 6, 1> v);

        Matrix<double, 6, 6> Mass();
        Matrix<double, 6, 6> Coreolis();
};