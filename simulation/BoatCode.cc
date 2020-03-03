#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <stdio.h>
#include <math.h>



#if GAZEBO_MAJOR_VERSION >= 9
  #include <ignition/math/Pose3.hh>
#else
  #include <gazebo/math/gzmath.hh>
#endif


#include <iostream>
#include <fstream>

#include "RMatrix.cc"
#include "Buoyancy.cc"
#include "Drag.cc"
#include "DynMod.cc"
#include "MassCoreolis.cc"
#include "Thrust.cc"
#include "Wind.cc"

//#include <eigen3/Eigen/Dense>
#include <eigen/Eigen/Dense>

namespace gazebo
{
    class BoatCode : public ModelPlugin
    {
        // Initialize program 
        public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
        {
            // Store the pointer to the model
            this->model = _parent;

            // Listen to the update event. This event is broadcast every
            // simulation iteration.
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                std::bind(&BoatCode::OnUpdate, this));

            // ================================================| vvv GO DOWN vvv |================================================
            // ================================================| vvv GO DOWN vvv |================================================
            // ================================================| vvv GO DOWN vvv |================================================
            //

            Ib(0, 0) = 1.0;
            Ib(1, 1) = 1.0;
            Ib(2, 2) = 1.0;

            rgb << -1, 0, 0;
            m = 1;

            for(int i = 0; i < MA.rows(); i++)
            {
                MA(i,i) = 1.0;
            }

            D = MA;

            rho = 10;
            nab = 1;
            GMT = 1;
            GML = 1;

            r << -1, 0, 0;
            f = 1.0;
            alfa = 0.0;

            rhoa = 1.25;
            AFw = 1.0;
            ALw = 1.0;
            HFw = 1.0;
            HLw = 1.0;
            Loa = 1.0;

            Vw = 0.0;
            betaw = 0.0;


            X(2) = 0.5;
        }

        // Called by the world update start event
        public: void OnUpdate()
        {
            std::cout << "===========================| " << counter << " |===========================" << std::endl << std::endl;
            counter++;

#if GAZEBO_MAJOR_VERSION >= 9
            // Hull
            this->model->GetLink("body")->SetWorldPose(ignition::math::Pose3d(X(0), X(1), X(2), X(3), X(4), X(5)));
            // Rudder
            this->model->GetLink("rudder")->SetWorldPose(ignition::math::Pose3d(X(0), X(1), (X(2)-0.5), X(3), X(4), (X(5)+3.14+alfa)));

#else
            // Hull
            this->model->GetLink("body")->SetWorldPose(math::Pose(X(0), X(1), X(2), X(3), X(4), X(5)));
            // Rudder
            this->model->GetLink("rudder")->SetWorldPose(math::Pose(X(0), X(1), (X(2)-0.5), X(3), X(4), (X(5)+3.14+alfa)));

#endif




            MC.Update(m, rgb, Ib, MA, V);
            M = MC.Mass();
            C = MC.Coreolis();

            Vr = V - Vc;

            DR.Update(D, Vr);
            DofVr = DR.DofVr();

            BOY.UpdateGeta(iAwp, rho, gravity, nab, GMT, GML, phi, th);
            gEta = BOY.Geta();

            BOY.UpdateG0(rho, gravity, Vi, xi, yi);
            g0 = BOY.G0();

            alfa = (M_PI/4)*sin(counter*sampleTime);

            THR.Update(r, f, alfa);
            tau = THR.Tau();

            WI.UpdateWindrw(V(3), V(4), Vw, betaw, psi);
            Vrw = WI.Vrw();

            WI.UpdateWind(rhoa, Vrw, Ci, AFw, ALw, HFw, HLw, Loa);
            tau_wind = WI.TauWind();

            cout << "Positions: \n" << X << endl;
            cout << "Velocities: \n" << V << endl;

            DM.Update(V, M, C, DofVr, gEta, g0, tau, tau_wind, tau_wave);
            
            V = V + DM.Vd()*sampleTime;

            X = X + V*sampleTime;
        }

        // Creating Function instances
        RMatrix RM;
        DynMod DM;
        MassCoreolis MC;
        Drag DR;
        Buoyancy BOY;
        Thrust THR;
        Wind WI;



        // Variables 
        private: 
            int counter = 0;

            // Dynamic Model variables 
            Eigen::Matrix<double, 6, 1> V = Eigen::Matrix<double, 6, 1>::Constant(0.0);
            Eigen::Matrix<double, 6, 1> X = Eigen::Matrix<double, 6, 1>::Constant(0.0);

            Eigen::Matrix<double, 6, 6> M = Eigen::Matrix<double, 6, 6>::Constant(0.0);
            Eigen::Matrix<double, 6, 6> C = Eigen::Matrix<double, 6, 6>::Constant(0.0);

            Eigen::Matrix<double, 6, 6> DofVr = Eigen::Matrix<double, 6, 6>::Constant(0.0);

            Eigen::Matrix<double, 6, 1> gEta = Eigen::Matrix<double, 6, 1>::Constant(0.0);
            Eigen::Matrix<double, 6, 1> g0 = Eigen::Matrix<double, 6, 1>::Constant(0.0);

            Eigen::Matrix<double, 6, 1> tau = Eigen::Matrix<double, 6, 1>::Constant(0.0);
            Eigen::Matrix<double, 6, 1> tau_wind = Eigen::Matrix<double, 6, 1>::Constant(0.0);
            Eigen::Matrix<double, 6, 1> tau_wave = Eigen::Matrix<double, 6, 1>::Constant(0.0);

            // General orientation 
            double phi = 0.0;
            double th = 0.0;
            double psi = 0.0;

            // Wind variables 
            double rhoa = 0.0;
            double Vrw = 0.0;
            Eigen::Matrix<double, 6, 1> Ci = Eigen::Matrix<double, 6, 1>::Constant(0.0);
            double AFw = 0.0;
            double ALw = 0.0;
            double HFw = 0.0;
            double HLw = 0.0;
            double Loa = 0.0;

            // Wind rw variables 
            double Vw = 0.0;
            double betaw = 0.0;

            // Thrust variables 
            Eigen::Vector3d r = Eigen::Vector3d::Constant(0.0);
            double f = 0.0;
            double alfa = 0.0;

            // Ballast variables 
            double rho = 0.0;
            Matrix<double, 2, 1> Vi = Eigen::Matrix<double, 2, 1>::Constant(0.0);
            Matrix<double, 2, 1> xi = Eigen::Matrix<double, 2, 1>::Constant(0.0);
            Matrix<double, 2, 1> yi = Eigen::Matrix<double, 2, 1>::Constant(0.0);

            // Buoyancy variables 
            double iAwp = 0.0;
            double nab = 0.0;
            double GMT = 0.0;
            double GML = 0.0;

            // Drag variables 
            Eigen::Matrix<double, 6, 1> Vr = Eigen::Matrix<double, 6, 1>::Constant(0.0);
            Eigen::Matrix<double, 6, 1> Vc = Eigen::Matrix<double, 6, 1>::Constant(0.0);
            Eigen::Matrix<double, 6, 6> D = Eigen::Matrix<double, 6, 6>::Constant(0.0);

            // Mass and Coreolis variables
            double m = 0.0;
            Eigen::Vector3d rgb = Eigen::Vector3d::Constant(0.0);
            Eigen::Matrix3d Ib = Eigen::Matrix3d::Constant(0.0);
            Eigen::Matrix<double, 6, 6> MA = Eigen::Matrix<double, 6, 6>::Constant(0.0);


            //double temperature = 20.0;
            //double pressure = 101325.0;


        // Constants 
        private: 
            const double sampleTime = 0.001;
            const double gravity = 9.81;
            //const double airGasConst = 287.058;


        //
        // ================================================| ^^^ GO UP ^^^ |================================================
        // ================================================| ^^^ GO UP ^^^ |================================================
        // ================================================| ^^^ GO UP ^^^ |================================================


        // Pointer to the model
        private: physics::ModelPtr model;

        // Pointer to the update event connection
        private: event::ConnectionPtr updateConnection;
    };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(BoatCode)
}
