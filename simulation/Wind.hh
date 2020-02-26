#include <stdio.h>
#include <math.h>

//#include <eigen3/Eigen/Dense>
#include <eigen/Eigen/Dense>

#include "RMatrix.hh"

using namespace std;
using namespace Eigen;

class Wind
{
    private: 
        /// 
        RMatrix RM;

        double Vrw_;
        double gamrw_;

        Matrix<double, 6, 1> tau;

        double uw;
        double vw;
        double urw;
        double vrw;

    public: 
        /// 
        Wind();

        void UpdateWind(double rhoa, double Vrw, Matrix<double, 6, 1> Ci, double AFw, double ALw, double HFw, double HLw, double Loa);
        void UpdateWindrw(double u, double v, double Vw, double betaw, double psi);

        Matrix<double, 6, 1> TauWind();
        double Vrw();
        double Gamrw();
};