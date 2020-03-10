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

#include "RMatrix.hh"
#include "Buoyancy.hh"
#include "Drag.hh"
#include "DynMod.hh"
#include "MassCoreolis.hh"
#include "Thrust.hh"
#include "Wind.hh"

//#include <eigen3/Eigen/Dense>
#include <eigen/Eigen/Dense>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>


namespace gazebo
{
    class BoatCode : public ModelPlugin
    {
        // Initialize program 
        public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
        {
			std::cout << "started";
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
			
			sub = n.subscribe("key_vel", 1, &BoatCode::updateinput, this);

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
            f = 0.0;
            alfa = 0.0;

            rhoa = 1.25;
            AFw = 1.0;
            ALw = 1.0;
            HFw = 1.0;
            HLw = 1.0;
            Loa = 1.0;

            Vw = 0.0;
            betaw = 0.0;
			/*
            // Open file 
            fileIsOpen = true;

            StateFile.open("State_log.csv");
            StateFile << "Log of states and positions.\n";
            StateFile << "x,y,z,phi,theta,psi,u,v,w,r,p,q\n";

            MFile.open("M_log.csv");
            MFile << "Log of added mass matrix\n";
            DC.WriteMatrix6header(MFile, M);

            TauFile.open("Tau_log.csv");
            TauFile << "Log of thrust vector\n";
            TauFile << "Tx, Ty, Tz, Mx, My, Mz\n";*/
        }

        // Called by the world update start event
        public: void OnUpdate()
        {
            std::cout << "===========================| " << counter << " |===========================" << std::endl << std::endl;
            counter++;

#if GAZEBO_MAJOR_VERSION >= 9
            // Hull
            this->model->GetLink("body")->SetWorldPose(ignition::math::Pose3d(Y(0), Y(1), Y(2)+0.5, Y(3), Y(4), (Y(5))));
            // Rudder
            this->model->GetLink("rudder")->SetWorldPose(ignition::math::Pose3d(Y(0), Y(1), Y(2), Y(3), Y(4), (Y(5)+3.14-alfa)));

#else
            // Hull
            this->model->GetLink("body")->SetWorldPose(ignition::math::Pose3d(Y(0), Y(1), Y(2)+0.5, Y(3), Y(4), (Y(5))));
            // Rudder
            this->model->GetLink("rudder")->SetWorldPose(ignition::math::Pose3d(Y(0), Y(1), Y(2), Y(3), Y(4), (Y(5)+3.14-alfa)));

#endif

            phi = X(3);
            th = X(4);
            psi = X(5);

            MC.Update(m, rgb, Ib, MA, V);
            M = MC.Mass();
            C = MC.Coreolis();

            Vr = V - Vc;

            DR.Update(D, Vr);
            DofVr = DR.DofVr();

            iAwp = X(2);

            BOY.UpdateGeta(iAwp, rho, gravity, nab, GMT, GML, phi, th);
            gEta = BOY.Geta();

            BOY.UpdateG0(rho, gravity, Vi, xi, yi);
            g0 = BOY.G0();

            //alfa = (M_PI/4)*sin(counter*sampleTime*0.25);

            THR.Update(r, f, alfa);
            tau = THR.Tau();

            WI.UpdateWindrw(V(0), V(1), Vw, betaw, psi);
            Vrw = WI.Vrw();

            Gamrw = WI.Gamrw();

            Ci = Gamrw*OneVec6;

            WI.UpdateWind(rhoa, Vrw, Ci, AFw, ALw, HFw, HLw, Loa);
            tau_wind = WI.TauWind();

            cout << "Positions: \n" << X << endl;
            cout << "Velocities: \n" << Etad << endl;

            DM.UpdateBODY(V, M, C, DofVr, gEta, g0, tau, tau_wind, tau_wave);

            J = RM.JofETa(phi, th, psi);
            Jd = (J - Jold)/sampleTime;
            Jold = J;

            DM.UpdateNED(J, Jd, Etad, M, C, DofVr, gEta, g0, tau, tau_wind, tau_wave);

            V = V + DM.Vd()*sampleTime;

            Etad = Etad + DM.Etadd()*sampleTime;

            X = X + Etad*sampleTime;

	    Y = RM.NED2Global(X);
			/*
            // Write to file 
            if(counter < 60000)
            {
                StateFile << X(0) << "," << X(1) << "," << X(2) << "," << X(3) << "," << X(4) << "," << X(5) << "," << 
                           Etad(0) << "," << Etad(1) << "," << Etad(2) << "," << Etad(3) << "," << Etad(4) << "," << Etad(5) << "\n";

                TauFile << tau(0) << "," << tau(1) << "," << tau(2) << "," << tau(3) << "," << tau(4) << "," << tau(5) << "\n";
            }
            else
            {
                if(fileIsOpen)
                {
                    DC.WriteMatrix6toFile(MFile, M);

                    fileIsOpen = false;

                    StateFile.close();
                    MFile.close();
                    TauFile.close();
                }
            }*/
            
        }
		
		void updateinput(const geometry_msgs::TwistConstPtr &msg)
        {
            f = msg->linear.x;
            alfa = msg-> angular.z;
            
        }

        // Creating Function instances
        RMatrix RM;
        DynMod DM;
        MassCoreolis MC;
        Drag DR;
        Buoyancy BOY;
        Thrust THR;
        Wind WI;

        // Debug instance 
        //DebugClass DC;

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

            // BODY to NED transformation variables
            Eigen::Matrix<double, 6, 1> Etad = Eigen::Matrix<double, 6, 1>::Constant(0.0);
            Eigen::Matrix<double, 6, 6> J = Eigen::Matrix<double, 6, 6>::Constant(0.0);
            Eigen::Matrix<double, 6, 6> Jd = Eigen::Matrix<double, 6, 6>::Constant(0.0);
            Eigen::Matrix<double, 6, 6> Jold = Eigen::Matrix<double, 6, 6>::Constant(0.0);

	    // NED to Global transformation variables
            Eigen::Matrix<double, 6, 1> Y = Eigen::Matrix<double, 6, 1>::Constant(0.0);

            // General orientation 
            double phi = 0.0;
            double th = 0.0;
            double psi = 0.0;

            // Wind variables 
            double rhoa = 0.0;
            double Vrw = 0.0;
            double Gamrw = 0.0;
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


            // Temporary variables 
            Eigen::Matrix<double, 6, 1> OneVec6 = Eigen::Matrix<double, 6, 1>::Constant(1.0);

            //double temperature = 20.0;
            //double pressure = 101325.0;
/*
            // File writing variables 
            bool fileIsOpen;

            ofstream StateFile;
            ofstream MFile;
            ofstream TauFile;*/

        // Constants 
        private: 
            const double sampleTime = 0.001;
            const double gravity = 9.81;
            //const double airGasConst = 287.058;
			
		private:
            ros::NodeHandle n;
            ros::Subscriber sub;


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
