#pragma once

#include <stdio.h>
#include <math.h>

#include <eigen/Eigen/Dense>

using namespace std;
using namespace Eigen;


class RMatrix
{
    private:
        Matrix3d M;

        Matrix3d R;
        Matrix3d vx;
        Matrix3d I;
        Matrix3d Z;

        Vector3d v;
        double s;
        double c;

        double thX;
        double thY;
        double thZ;

        Vector3d Vec;

        Matrix<double, 6, 6> J;
    public: 
        // Constructor
        RMatrix();
 
        double pi;

        Matrix3d Rx(double x);
        Matrix3d Ry(double x);
        Matrix3d Rz(double x);

        Matrix3d GetRotMat(Vector3d a, Vector3d b);
        int Sign(double x);
        Vector3d R_ZYX_2EUL(Matrix3d rm);

        Matrix3d SofL(Vector3d L);
        Matrix<double, 6, 6> JofETa(double phi, double th, double psi);
	Matrix<double, 6, 1> NED2Global(Matrix<double, 6, 1> Eta);
};
